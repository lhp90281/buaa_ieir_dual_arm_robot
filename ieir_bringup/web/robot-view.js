import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';
import { STLLoader } from '/vendor/STLLoader.js';
import { PoseBuffer } from './pose-buffer.js';

export class RobotView {
  constructor(element, model) {
    this.element = element;
    this.model = model;
    this.groups = new Map();
    this.joints = new Map();
    this.poseBuffer = new PoseBuffer();
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xedf1f3);
    this.camera = new THREE.PerspectiveCamera(38, 1, 0.01, 50);
    this.camera.up.set(0, 0, 1);
    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 1.25));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    element.prepend(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.minDistance = 0.3;
    this.controls.maxDistance = 8;
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x798587, 1.25));
    const light = new THREE.DirectionalLight(0xffffff, 2.0);
    light.position.set(-2, -4, 6);
    this.scene.add(light);
    const fill = new THREE.DirectionalLight(0xd6e9ed, 0.6);
    fill.position.set(3, 3, 2);
    this.scene.add(fill);
    this.grid = new THREE.GridHelper(4, 40, 0xadbfc5, 0xd2dce0);
    this.grid.rotation.x = Math.PI / 2;
    this.grid.position.z = -0.004;
    this.grid.material.transparent = true;
    this.grid.material.opacity = 0.55;
    this.scene.add(this.grid);
    this.axes = new THREE.AxesHelper(0.25);
    this.axes.visible = false;
    this.scene.add(this.axes);
    this.root = new THREE.Group();
    this.scene.add(this.root);
    const loader = new STLLoader();
    const meshes = [];
    for (const link of model.links) {
      const group = new THREE.Group();
      group.name = link.name;
      this.groups.set(link.name, group);
      for (const visual of link.visuals) {
        meshes.push(loader.loadAsync(visual.mesh).then(geometry => {
          const accent = /base_link$/.test(link.name) && /^(left|right)/.test(link.name);
          const color = accent ? (link.name.startsWith('left') ? 0x488ea6 : 0xbd9656) : 0xc8d1d5;
          const mesh = new THREE.Mesh(geometry, new THREE.MeshPhongMaterial({
            color, specular: 0x718087, shininess: 35,
          }));
          mesh.position.fromArray(visual.xyz);
          mesh.quaternion.setFromEuler(new THREE.Euler(...visual.rpy, 'ZYX'));
          mesh.scale.fromArray(visual.scale);
          mesh.updateMatrix();
          mesh.matrixAutoUpdate = false;
          group.add(mesh);
        }));
      }
    }
    const children = new Set();
    for (const joint of model.joints) {
      const origin = new THREE.Group();
      origin.position.fromArray(joint.xyz);
      origin.quaternion.setFromEuler(new THREE.Euler(...joint.rpy, 'ZYX'));
      origin.updateMatrix();
      origin.matrixAutoUpdate = false;
      const motion = new THREE.Group();
      origin.add(motion);
      motion.add(this.groups.get(joint.child));
      this.groups.get(joint.parent).add(origin);
      children.add(joint.child);
      if (joint.type !== 'fixed') this.joints.set(joint.name, {
        group: motion, joint, axis: new THREE.Vector3(...joint.axis).normalize(),
      });
    }
    for (const [name, group] of this.groups) if (!children.has(name)) this.root.add(group);
    this.resize = new ResizeObserver(() => {
      const width = element.clientWidth, height = element.clientHeight;
      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(width, height);
    });
    this.resize.observe(element);
    this.fit();
    this.ready = Promise.all(meshes).then(() => {
      this.fit();
      return meshes.length;
    });
    this.frames = 0;
    this.renderer.setAnimationLoop(now => {
      if (document.hidden) return;
      const positions = this.poseBuffer.sample(now);
      if (positions) this.applyPose(positions);
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
      this.frames++;
    });
  }
  fit() {
    const box = new THREE.Box3().setFromObject(this.root);
    const center = box.isEmpty() ? new THREE.Vector3(0, 0, 0.7) : box.getCenter(new THREE.Vector3());
    const size = box.isEmpty() ? 1.5 : box.getSize(new THREE.Vector3()).length();
    const distance = Math.max(1.9, size * 1.55 / Math.min(1, this.camera.aspect));
    this.camera.position.copy(center).add(new THREE.Vector3(0.75, -1.7, 0.8).normalize().multiplyScalar(distance));
    this.controls.target.copy(center);
    this.controls.update();
  }
  setPose(positions) {
    this.poseBuffer.clear();
    this.applyPose(positions);
  }
  pushPose(positions, now = performance.now()) {
    this.poseBuffer.push(positions, now);
  }
  applyPose(positions) {
    for (const [name, value] of Object.entries(positions)) {
      const entry = this.joints.get(name);
      if (!entry || !Number.isFinite(value)) continue;
      if (entry.joint.type === 'prismatic') entry.group.position.copy(entry.axis).multiplyScalar(value);
      else entry.group.quaternion.setFromAxisAngle(entry.axis, value);
    }
  }
}

export const quaternionFromRPY = values => new THREE.Quaternion()
  .setFromEuler(new THREE.Euler(...values, 'ZYX')).toArray();
export const rpyFromQuaternion = values => {
  const e = new THREE.Euler().setFromQuaternion(new THREE.Quaternion(...values), 'ZYX');
  return [e.x, e.y, e.z];
};
