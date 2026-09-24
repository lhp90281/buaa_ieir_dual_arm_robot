import { RobotView, quaternionFromRPY, rpyFromQuaternion } from './robot-view.js';

const $ = selector => document.querySelector(selector);
const $$ = selector => [...document.querySelectorAll(selector)];
const deg = value => value * 180 / Math.PI;
const rad = value => value * Math.PI / 180;
const fmt = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '--';
const labels = { overview: '运行总览', joints: '关节控制', cartesian: '末端控制', teleop: '主从遥操作', calibration: '关节标定', recording: '示教记录', config: '运行配置' };
const names = { gravity_compensation_controller: '重力补偿', joint_position_controller: '关节位置', cartesian_position_controller: '笛卡尔位置' };
let bootstrap, state, config, viewer, arm = 'left', tableArm = 'left', view = 'overview', sceneMode = 'live';
let targets = {}, connected = false, pending = false, clearedThrough = 0, toastTimer;
let lastLogSignature = '', lastCalSignature = '', poseCaptured = false;
let poseStream, latestPositions = {}, poseStreamAt = 0;
let calAngleJoint = '';
let processSignature = '';

function element(tag, text, className) {
  const e = document.createElement(tag);
  if (text !== undefined) e.textContent = text;
  if (className) e.className = className;
  return e;
}
function icons() { window.lucide?.createIcons(); }
function toast(message, error = false) {
  const e = $('#toast');
  e.textContent = message;
  e.className = error ? 'error' : '';
  e.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { e.hidden = true; }, error ? 10000 : 5000);
}
async function api(path, options) {
  const response = await fetch(path, { cache: 'no-store', ...options });
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}
function confirm(title, message) {
  const dialog = $('#confirm-dialog');
  $('#confirm-title').textContent = title;
  $('#confirm-message').textContent = message;
  return new Promise(resolve => {
    dialog.returnValue = 'cancel';
    dialog.addEventListener('close', () => resolve(dialog.returnValue === 'confirm'), { once: true });
    dialog.showModal();
  });
}
async function command(action, payload, title, warning, readOnlyAction = false) {
  if (pending || !connected) return toast('当前连接不可用或有操作正在等待结果', true);
  if (!readOnlyAction && !$('#unlock').checked) return toast('当前为只读模式', true);
  if (title && !(await confirm(title, warning))) return;
  if (!connected || (!readOnlyAction && !$('#unlock').checked)) return toast('连接或控制权限已改变，请重新确认', true);
  pending = true;
  updateControls();
  $('#footer-status').textContent = '正在等待操作结果…';
  try {
    const result = await api('/api/action', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', 'X-Console-Token': bootstrap.token },
      body: JSON.stringify({ ...payload, action, confirmed: true }),
    });
    toast(result.message);
    await poll();
    if (action === 'record_stop') await loadConfig();
    if (action === 'process_start' && payload.kind === 'calibration') {
      sceneMode = 'calibration';
      $$('[data-scene]').forEach(b => b.classList.toggle('selected', b.dataset.scene === sceneMode));
      renderScene(true);
    }
  } catch (error) { toast(error.message, true); }
  finally { pending = false; updateControls(); }
}

function updateControls() {
  const enabled = connected && !pending && $('#unlock').checked && bootstrap?.allow_control;
  const fresh = Object.values(state?.joints || {}).some(j => j.age < 0.5);
  const active = name => state?.controller_age < 3 && state?.controllers[name] === 'active';
  $$('[data-mode]').forEach(b => { b.disabled = !enabled || !fresh; });
  $('#send-joints').disabled = !enabled || !active('joint_position_controller') || !active('gravity_compensation_controller') || !fresh;
  $$('[data-zero]').forEach(b => {
    const sides = b.dataset.zero === 'dual' ? ['left', 'right'] : [b.dataset.zero];
    b.disabled = !enabled || state?.controller_age >= 3 || !sides.every(side =>
      Array.from({ length: 7 }, (_, i) => state?.joints[`${side}_joint_${i}`]?.age < 0.5).every(Boolean));
  });
  $('#send-pose').disabled = !enabled || !poseCaptured || !active('cartesian_position_controller') || !fresh;
  $$('[data-teleop]').forEach(b => { b.disabled = !enabled || !state?.services.includes(`/teleop/${b.dataset.teleop}`); });
  $$('[data-key]').forEach(b => { b.disabled = !enabled || !$('#cal-session').value; });
  $('#set-cal-angle').disabled = !enabled || !$('#cal-session').value;
  const running = kind => state?.processes?.[kind]?.running;
  const anyProcess = Object.values(state?.processes || {}).some(p => p.running);
  $$('[data-start-kind]').forEach(b => {
    const kind = b.dataset.startKind;
    const busy = ['control', 'calibration'].includes(kind) ? anyProcess : ['teleop', 'replay', 'calibration', 'merge'].some(running);
    b.disabled = !enabled || !bootstrap?.manage_processes || busy;
  });
  $$('[data-stop-kind]').forEach(b => { b.disabled = !enabled || !running(b.dataset.stopKind); });
  $('#record-start').disabled = !connected || pending || !fresh || !!state?.recording;
  $('#record-stop').disabled = !connected || pending || !state?.recording;
  $('#capture-joints').disabled = !connected || !fresh;
  $('#capture-pose').disabled = !connected || !(state?.poses[arm]?.age < 0.5);
  $('#unlock').disabled = !bootstrap?.allow_control || !connected;
  $('#safety-text').textContent = $('#unlock').checked ? '控制已解锁 · 每次下发仍需确认' : '只读连接 · 未授权网页发送控制命令';
  $('.safety-strip').classList.toggle('armed', $('#unlock').checked);
  if (!pending) $('#footer-status').textContent = !connected ? '连接中断 · 控制已锁定' : bootstrap?.preview ? '离线预览 · 不连接 ROS / 电机' : $('#unlock').checked ? '本地控制已解锁' : '本地连接 · 只读';
}

function buildJointEditors() {
  const container = $('#joint-inputs');
  container.replaceChildren();
  for (let i = 0; i < 7; i++) {
    const name = `${arm}_joint_${i}`;
    const limit = bootstrap.model.joints.find(j => j.name === name);
    if (!(name in targets)) targets[name] = state?.joints[name]?.position ?? 0;
    const row = element('div', undefined, 'joint-editor');
    const label = element('label', `J${i + 1}`); label.htmlFor = `joint-number-${i}`;
    const slider = element('input'); slider.type = 'range'; slider.min = deg(limit.lower); slider.max = deg(limit.upper); slider.step = '0.1';
    slider.setAttribute('aria-label', `${arm} 关节 ${i + 1} 滑块`);
    const input = element('input'); input.id = `joint-number-${i}`; input.type = 'number'; input.min = deg(limit.lower).toFixed(1); input.max = deg(limit.upper).toFixed(1); input.step = '0.1';
    input.setAttribute('aria-label', `${arm} 关节 ${i + 1} 角度`);
    input.value = slider.value = fmt(deg(targets[name]), 1);
    slider.addEventListener('input', () => { input.value = slider.value; targets[name] = rad(Number(slider.value)); renderScene(); });
    input.addEventListener('input', () => { if (Number.isFinite(input.valueAsNumber)) { slider.value = input.value; targets[name] = rad(input.valueAsNumber); renderScene(); } });
    row.append(label, slider, input); container.append(row);
  }
}
function captureJoints() {
  for (const [name, joint] of Object.entries(state?.joints || {})) if (joint.age < 0.5) targets[name] = joint.position;
  buildJointEditors(); renderScene();
}
function renderScene(reset = false) {
  if (!viewer) return;
  const live = Object.fromEntries(Object.entries(state?.joints || {}).map(([n, j]) => [n, j.position]));
  let positions = live;
  if (sceneMode === 'target') positions = { ...live, ...targets };
  if (sceneMode === 'calibration') {
    positions = Object.fromEntries(bootstrap.model.joints.map(j => [j.name, 0]));
    Object.assign(positions, state?.calibration[$('#cal-session').value]?.positions || {});
  }
  if (sceneMode !== 'live') viewer.setPose(positions);
  else if (reset) viewer.setPose(latestPositions);
  $('#pose-label').textContent = sceneMode === 'target' ? '目标预览 · 未自动下发' : sceneMode === 'calibration' ? '标定参考 / 采样后预览' : performance.now() - poseStreamAt < 500 && Object.keys(latestPositions).length ? bootstrap.preview ? '合成数据 · 非真机' : '实测姿态' : '无新反馈 · 显示非实时';
}

function startPoseStream() {
  poseStream?.close();
  poseStream = new EventSource('/api/pose-stream');
  poseStream.onmessage = event => {
    latestPositions = JSON.parse(event.data).positions;
    poseStreamAt = performance.now();
    if (sceneMode === 'live') viewer?.pushPose(latestPositions, poseStreamAt);
  };
  poseStream.onerror = () => {
    poseStreamAt = 0;
    $('#unlock').checked = false;
    updateControls();
  };
}
function renderTable() {
  const body = $('#joint-table'); body.replaceChildren();
  for (let i = 0; i < 7; i++) {
    const j = state?.joints[`${tableArm}_joint_${i}`];
    const motor = state?.motors.find(m => m.channel === (tableArm === 'left' ? 0 : 1) && m.slot === i);
    const row = element('tr');
    const name = element('td'); const label = element('span', undefined, 'joint-name');
    label.append(element('span', `${i + 1}`, 'joint-index'), element('span', `${tableArm}_joint_${i}`)); name.append(label); row.append(name);
    row.append(element('td', fmt(j ? deg(j.position) : null)), element('td', fmt(j?.velocity == null ? null : deg(j.velocity))), element('td', fmt(j?.effort, 3)));
    const online = motor?.online && motor.age < 0.5;
    row.append(element('td', online ? motor.error > 1 ? `错误 ${motor.error}` : motor.enabled ? '已使能' : '失能' : '无反馈', online ? motor.error > 1 ? 'danger-text' : 'ok-text' : 'muted'));
    if (!j || j.age > 0.5) row.classList.add('muted');
    body.append(row);
  }
}
function renderState() {
  const values = Object.values(state.joints);
  const fresh = values.filter(j => j.age < 0.5);
  $('#joint-count').replaceChildren(document.createTextNode(`${fresh.length} `), element('small', '/ 14'));
  const minAge = fresh.length ? Math.max(...fresh.map(j => j.age)) * 1000 : null;
  $('#age-status').replaceChildren(document.createTextNode(`${fmt(minAge, 0)} `), element('small', 'ms'));
  for (const [side, channel] of [['left', 0], ['right', 1]]) {
    const online = state.motors.filter(m => m.channel === channel && m.slot < 7 && m.online && m.age < 0.5).length;
    $(`#${side}-status`).textContent = `${online} / 7 在线`;
  }
  const active = Object.entries(state.controllers).filter(([_, value]) => value === 'active').map(([name]) => name);
  const current = state.controller_age > 3 ? '状态过期' : active.includes('cartesian_position_controller') ? '笛卡尔位置' : active.includes('joint_position_controller') ? '关节位置' : active.includes('gravity_compensation_controller') ? '重力补偿' : '未激活';
  $('#mode-status').textContent = current;
  const controllers = $('#controllers'); controllers.replaceChildren();
  for (const [name, title] of Object.entries(names)) {
    const isActive = state.controllers[name] === 'active' && state.controller_age < 3;
    const row = element('div', undefined, 'controller-row');
    const text = element('div'); text.append(element('strong', title), element('small', name));
    row.append(element('span', undefined, isActive ? 'active-dot' : ''), text,
      element('span', state.controller_age > 3 ? '未确认' : isActive ? 'ACTIVE' : state.controllers[name] === 'inactive' ? 'INACTIVE' : '未加载', `pill ${isActive ? 'ok' : ''}`));
    controllers.append(row);
  }
  $('#node-count').textContent = `${state.nodes.length} NODES`;
  for (const [id, ok] of [['bridge', state.motors.some(m => m.online && m.age < .5)], ['joint', fresh.length > 0], ['manager', state.controller_age < 3]]) {
    $(`#${id}-pill`).textContent = ok ? '已连接' : '等待'; $(`#${id}-pill`).classList.toggle('ok', ok);
  }
  const poses = $('#pose-readouts'); poses.replaceChildren();
  for (const side of ['left', 'right']) {
    const p = state.poses[side];
    const row = element('div', undefined, 'pose-readout'); const label = element('span');
    label.append(element('b', undefined, `${side}-dot`), document.createTextNode(side === 'left' ? '左臂' : '右臂'));
    row.append(label, element('span', p && p.age < 0.5 ? p.position.map(v => fmt(v, 3)).join('  ') : '等待 TF')); poses.append(row);
  }
  const services = state.services.filter(s => s.startsWith('/teleop/'));
  $('#teleop-state').textContent = services.length ? `${services.length} 个遥操作服务可用 · 对端状态待服务确认` : '未发现遥操作服务';
  const signature = Object.keys(state.calibration).join(',');
  if (signature !== lastCalSignature) {
    const select = $('#cal-session'); const previous = select.value; select.replaceChildren();
    if (!signature) select.append(new Option('没有活动会话', ''));
    Object.keys(state.calibration).forEach((key, index) => select.append(new Option(`会话 ${index + 1} · ${key.slice(-8)}`, key)));
    if (state.calibration[previous]) select.value = previous;
    lastCalSignature = signature;
  }
  $('#cal-status').textContent = state.calibration[$('#cal-session').value]?.text || '等待标定状态';
  const meta = state.calibration[$('#cal-session').value]?.metadata || {};
  $('#cal-angle-row').hidden = meta.stage !== 'limits' || !meta.joint;
  const angleJoint = `${$('#cal-session').value}:${meta.joint}`;
  if (angleJoint !== calAngleJoint) {
    $('#cal-angle').value = Number.isFinite(meta.angle_deg) ? meta.angle_deg.toFixed(2) : '';
    calAngleJoint = angleJoint;
  }
  const processes = $('#process-status'); processes.replaceChildren();
  const newSignature = JSON.stringify(state.processes || {});
  if (newSignature !== processSignature) {
    if (processSignature) loadConfig();
    processSignature = newSignature;
  }
  for (const process of Object.values(state.processes || {})) {
    const row = element('div', undefined, 'file-row');
    row.append(element('span', process.label), element('span', process.running ? process.stopping ? '停止中' : '进程运行' : `已退出 (${process.exit_code})`, process.running ? 'ok-text' : 'muted'));
    processes.append(row);
  }
  const record = state.recording;
  $('#record-label').textContent = record ? '正在记录' : '未录制';
  const seconds = record?.duration || 0;
  $('#record-time').textContent = `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${(seconds % 60).toFixed(1).padStart(4, '0')}`;
  $('#record-count').textContent = `${record?.count || 0} 帧`;
  renderTable(); renderLogs(); renderScene(); updateControls();
}
function renderLogs() {
  const filter = $('#log-filter').value;
  const rows = (state?.logs || []).filter(l => l.id > clearedThrough && (filter === 'all' || filter === 'error' ? filter === 'all' || l.level === 'error' : l.level !== 'info')).slice(-80);
  const signature = `${filter}:${clearedThrough}:${rows.at(-1)?.id}`;
  if (signature === lastLogSignature) return;
  lastLogSignature = signature;
  const log = $('#logs'); const nearBottom = log.scrollHeight - log.scrollTop - log.clientHeight < 40;
  log.replaceChildren();
  if (!rows.length) log.append(element('div', '没有符合条件的日志', 'log-empty'));
  for (const row of rows) {
    const line = element('div', undefined, `log-line ${row.level}`);
    line.append(element('span', row.time), element('span', row.level.toUpperCase()), element('span', row.source, 'log-source'), element('span', row.text, 'log-message'));
    log.append(line);
  }
  if (nearBottom) log.scrollTop = log.scrollHeight;
}
async function poll() {
  try {
    state = await api('/api/state'); connected = true;
    $('#connection').textContent = '本地已连接'; $('#connection').className = 'badge good';
    renderState();
  } catch (error) {
    connected = false; $('#unlock').checked = false;
    $('#connection').textContent = '连接中断'; $('#connection').className = 'badge bad'; updateControls();
  }
}
async function polling() { if (!document.hidden) await poll(); setTimeout(polling, 250); }
async function heartbeat() {
  if (!document.hidden && connected && window.consoleMeshCount) {
    try {
      await api('/api/action', { method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Console-Token': bootstrap.token },
        body: JSON.stringify({ action: 'heartbeat', confirmed: true }) });
    } catch { /* The calibration node independently times out absent heartbeats. */ }
  }
  setTimeout(heartbeat, 500);
}
async function loadConfig() {
  try {
    config = await api('/api/config');
    const files = $('#config-files'); files.replaceChildren();
    for (const file of config.files) {
      const row = element('div', undefined, 'file-row');
      row.append(element('span', file.name), element('span', file.status === 'readable' ? file.entries ? `${file.entries} 项` : '可读取' : file.status === 'missing' ? '未生成' : '解析错误', file.status === 'readable' ? 'ok-text' : 'warn-text')); files.append(row);
    }
    const recordings = $('#recordings'); recordings.replaceChildren();
    if (!config.recordings.length) recordings.append(element('div', '暂无网页录制轨迹', 'empty-state'));
    for (const file of config.recordings) {
      const row = element('div', undefined, 'file-row');
      const button = element('button', undefined, 'icon'); button.title = '复制原控制器回放命令'; button.setAttribute('aria-label', button.title); button.innerHTML = '<i data-lucide="copy"></i>';
      button.disabled = !file.replay;
      if (!file.replay) button.title = '预览合成数据，不提供真机回放';
      button.onclick = () => file.replay && copy(file.replay);
      row.append(element('span', file.name), element('span', `${Math.round(file.bytes/1024)} KB`, 'muted'), button);
      if (bootstrap.manage_processes && file.replay) {
        const play = element('button', undefined, 'icon'); play.title = '回放轨迹'; play.setAttribute('aria-label', play.title);
        play.innerHTML = '<i data-lucide="play"></i>'; play.dataset.startKind = 'replay';
        play.onclick = () => command('process_start', { kind: 'replay', file: file.name }, '回放此轨迹？',
          `${file.name}\n0.5 倍速，5 s 进入起点。会驱动所记录关节，无碰撞检查。请确认起点、路径与支撑状态。`);
        row.append(play);
      }
      recordings.append(row);
    }
    updateCalCommand(); icons();
  } catch (error) { toast(error.message, true); }
}
function updateCalCommand() {
  $('#cal-command').textContent = config?.commands[arm][$('#cal-stage').value] || '';
  $('#cal-joint-row').hidden = $('#cal-stage').value !== 'manual';
}
async function copy(text) {
  try { await navigator.clipboard.writeText(text); toast('命令已复制'); }
  catch { toast('剪贴板不可用，请选中命令文本复制', true); }
}
function setArm(side) {
  arm = side; poseCaptured = false;
  $$('[data-arm]').forEach(b => b.classList.toggle('selected', b.dataset.arm === arm));
  buildJointEditors(); updateCalCommand(); updateControls();
  $$('#pose-inputs input').forEach(e => { e.value = ''; });
}

async function init() {
  icons();
  bootstrap = await api('/api/bootstrap');
  $$('.managed-section').forEach(e => { e.hidden = !bootstrap.manage_processes; });
  $('#environment').hidden = !bootstrap.preview;
  $('#model-tag').textContent = bootstrap.model.gripper ? '含夹爪' : '无夹爪';
  $('#workspace-path').textContent = bootstrap.workspace;
  $('#config-model').textContent = `固定基座 · 双臂 · ${bootstrap.model.gripper ? '含夹爪' : '无夹爪'}`;
  $('#unlock').title = bootstrap.allow_control ? '本次页面控制权限' : '服务端只读；需要以 --allow-control 启动';
  try {
    viewer = new RobotView($('#viewport'), bootstrap.model);
    window.consoleScene = viewer;
    viewer.ready.then(count => { $('#model-loading').hidden = true; window.consoleMeshCount = count; }).catch(error => { $('#model-loading').textContent = `模型加载失败：${error.message}`; });
  } catch (error) { $('#model-loading').textContent = `WebGL 不可用：${error.message}`; }
  buildJointEditors();
  for (const [index, title] of ['X / m', 'Y / m', 'Z / m', 'Roll / °', 'Pitch / °', 'Yaw / °'].entries()) {
    const label = element('label', title); const input = element('input');
    input.id = `pose-${index}`; input.type = 'number'; input.step = index < 3 ? '0.001' : '0.1';
    input.setAttribute('aria-label', title); label.append(input); $('#pose-inputs').append(label);
  }
  $$('.nav').forEach(b => b.onclick = () => {
    view = b.dataset.view;
    $$('.nav').forEach(n => n.classList.toggle('active', n === b));
    $$('.view').forEach(v => { v.hidden = v.id !== `view-${view}`; });
    $('#page-title').textContent = labels[view];
    if (['config', 'calibration', 'recording'].includes(view)) loadConfig();
  });
  $$('[data-scene]').forEach(b => b.onclick = () => {
    sceneMode = b.dataset.scene; $$('[data-scene]').forEach(n => n.classList.toggle('selected', n === b)); renderScene(true);
  });
  $$('[data-table]').forEach(b => b.onclick = () => { tableArm = b.dataset.table; $$('[data-table]').forEach(n => n.classList.toggle('selected', n === b)); renderTable(); });
  $$('[data-arm]').forEach(b => b.onclick = () => setArm(b.dataset.arm));
  $('#fit-view').onclick = () => viewer?.fit();
  $('#toggle-grid').onclick = e => { if (viewer) { viewer.grid.visible = !viewer.grid.visible; e.currentTarget.classList.toggle('selected', viewer.grid.visible); } };
  $('#toggle-axes').onclick = e => { if (viewer) { viewer.axes.visible = !viewer.axes.visible; e.currentTarget.classList.toggle('selected', viewer.axes.visible); } };
  $('#unlock').onchange = async () => {
    if ($('#unlock').checked) {
      $('#unlock').checked = false;
      const accepted = await confirm('开启本次页面控制权限？', bootstrap.preview ? '当前是隔离预览，不连接 ROS 或电机。' : '确认已核实连接的机器人、物理急停与支撑状态。后续操作可能切换控制器或驱动机械臂，网页关闭不会自动停止既有 ROS 控制。');
      $('#unlock').checked = accepted && connected;
    }
    updateControls();
  };
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) { poseStream?.close(); $('#unlock').checked = false; updateControls(); }
    else startPoseStream();
  });
  $$('[data-mode]').forEach(b => b.onclick = () => command('switch', { mode: b.dataset.mode }, `切换到${b.textContent.trim()}？`, '保持重力前馈，仅切换位置接口的控制器。请确认无人占用机械臂扫掠区域。'));
  $('#start-control').onclick = () => command('process_start', { kind: 'control', arms: $('#runtime-arms').value },
    '启动真机控制端？', '将加载本机标定和摩擦文件，并自动使能所选机械臂、进入重力补偿。请确认标定正确、夹爪装配设置一致、支撑和急停就绪。');
  $$('[data-stop-kind]').forEach(b => b.onclick = () => command('process_stop', { kind: b.dataset.stopKind },
    '停止此进程？', b.dataset.stopKind === 'control' || b.dataset.stopKind === 'calibration'
      ? '停止后会尝试失能电机，机械臂失去支撑力。请先物理支撑全部关节。Bridge 保持运行；仅停止本网页启动的进程。'
      : '退出目标发布进程，等待清理完成。不停止 bridge，不等同于物理急停。'));
  $('#start-teleop').onclick = () => command('process_start', { kind: 'teleop', role: $('#teleop-role').value,
    peer_host: $('#teleop-host').value.trim(), local_port: $('#teleop-local-port').valueAsNumber,
    peer_port: $('#teleop-peer-port').valueAsNumber }, '启动主从遥操作节点？', '使用 no_feedback 模式。启动后仍需分别确认准备对齐、开始跟随；两台机器使用不同 ROS_DOMAIN_ID。');
  $('#start-calibration').onclick = () => {
    const stage = $('#cal-stage').value;
    command('process_start', { kind: stage === 'merge' ? 'merge' : 'calibration', side: arm, stage,
      joint: $('#cal-joint').value ? Number($('#cal-joint').value) : null }, '启动所选标定阶段？',
      stage === 'merge' ? '合并左右各 7 个关节到本机 joint_offsets_dual.yaml，旧文件先备份。完成后重新启动控制端读取，无需重新编译。'
      : stage === 'limits' ? '仅显示和修改理论参考角，不发送电机命令。确认全部 7 个关节后保存 reviewed 文件。'
      : '请先停止控制端并支撑所有关节。方向确认将使能所选整臂；手动采样只使能当前关节，零增益、零力矩。不会自动寻限位或回零。保持本页面可见。');
  };
  $('#set-cal-angle').onclick = () => command('calibration_angle', { session: $('#cal-session').value,
    angle_deg: $('#cal-angle').valueAsNumber }, '更新机械限位参考角？', '仅更新预览角度，之后仍需确认此关节。请按实际机械限位核实正负与度数。');
  $('#capture-joints').onclick = captureJoints;
  $$('[data-zero]').forEach(b => b.onclick = () => {
    const duration = $('#zero-duration').valueAsNumber;
    command('zero', { side: b.dataset.zero, duration }, `${b.textContent.trim()}？`,
      `所选关节将回到标定后的模型 0°，不是重新标定电机。\n自动切换到关节位置控制，保留重力补偿。请求 ${duration} s，距离较大时自动延长至峰值不超过 20°/s。\n可能大幅运动，没有碰撞检查，请确认整个回零路径。`);
  });
  $('#send-joints').onclick = () => {
    const positions = Object.fromEntries(Object.entries(targets).filter(([name]) => name.startsWith(arm)));
    const duration = $('#duration').valueAsNumber;
    command('joint', { positions, duration }, `发送${arm === 'left' ? '左臂' : '右臂'}关节目标？`, `${duration} s 插值。目标（度）：\n${Object.values(positions).map(q => fmt(deg(q), 1)).join(' / ')}\n没有碰撞规划，请确认路径。`);
  };
  $('#capture-pose').onclick = () => {
    const pose = state.poses[arm]; if (!pose || pose.age > .5) return toast('缺少实时末端 TF', true);
    [...pose.position, ...rpyFromQuaternion(pose.quaternion).map(deg)].forEach((v, i) => { $(`#pose-${i}`).value = fmt(v, i < 3 ? 4 : 2); });
    poseCaptured = true; updateControls();
  };
  $('#send-pose').onclick = () => {
    const values = Array.from({ length: 6 }, (_, i) => $(`#pose-${i}`).valueAsNumber);
    if (!values.every(Number.isFinite)) return toast('请填写完整的末端位姿', true);
    command('cartesian', { side: arm, position: values.slice(0, 3), quaternion: quaternionFromRPY(values.slice(3).map(rad)) }, '发送末端目标？', `坐标系 base_footprint\nXYZ = ${values.slice(0, 3).map(v => fmt(v, 4)).join(', ')} m\n由现有 IK 控制器执行，不保证末端直线轨迹。`);
  };
  $$('[data-teleop]').forEach(b => b.onclick = () => command('teleop', { operation: b.dataset.teleop }, `${b.querySelector('span').firstChild.textContent}？`, b.dataset.teleop === 'prepare' ? '准备流程将让主臂自动对齐从臂。确认两台机器的扫掠区域与急停。' : '此操作作用于已有遥操作节点。暂停或退出不等于电机失能。'));
  $('#cal-stage').onchange = updateCalCommand;
  $('#copy-cal').onclick = () => copy($('#cal-command').textContent);
  $('#cal-session').onchange = () => { renderState(); };
  $$('[data-key]').forEach(b => b.onclick = () => command('calibration_key', { key: b.dataset.key, session: $('#cal-session').value }, `标定：${b.textContent}？`, `${$('#cal-status').textContent}\n\n确认仅当前标定会话持有电机控制权，关节已支撑。`));
  $('#record-start').onclick = () => command('record_start', {}, null, null, true);
  $('#record-stop').onclick = () => command('record_stop', {}, null, null, true);
  $('#refresh-config').onclick = $('#refresh-recordings').onclick = loadConfig;
  $('#log-filter').onchange = renderLogs;
  $('#clear-logs').onclick = () => { clearedThrough = state?.logs.at(-1)?.id || 0; renderLogs(); };
  await loadConfig();
  startPoseStream();
  polling();
  heartbeat();
}
init().catch(error => { $('#connection').textContent = '初始化失败'; $('#model-loading').textContent = error.message; toast(error.message, true); });
