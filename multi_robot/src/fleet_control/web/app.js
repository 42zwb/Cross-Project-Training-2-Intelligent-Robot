'use strict';
(() => {
  const $ = id => document.getElementById(id);
  const colors = ['#56bbff', '#ffc16f', '#b49cff'];
  const ctx = $('map').getContext('2d');
  const labels = {BOOTSTRAP:'正在启动', IDLE:'待命', SWITCHING:'切换任务', EXPLORING:'协同探索', EXPLORED:'探索完成', RETURNING:'返回分区', FORMING:'队形调整', FORMATION_READY:'队形就绪', PATROLLING:'航点巡逻', PATROL_DONE:'巡逻完成', ESTOP:'急停锁定', FAULT:'故障停车'};
  let state = {}, image = null, version = -1, initialized = false, connected = false, pending = false, requestSequence = 0;

  async function fetchJSON(path, options = {}, timeout = 15000) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeout);
    try {
      const response = await fetch(path, {...options, signal: controller.signal, cache: 'no-store'});
      const data = await response.json();
      if (!response.ok) throw new Error(data.message || 'HTTP ' + response.status);
      return data;
    } finally { clearTimeout(timer); }
  }
  function feedback(message, success = false) {
    $('response').textContent = message;
    $('response').className = success ? 'success' : 'error';
  }
  function updateControls() {
    document.querySelectorAll('[data-motion]').forEach(b => {
      const ready = state.architecture === 'decentralized' && b.dataset.command === 'explore' ? state.local_ready :
        state.architecture === 'decentralized' && b.id === 'start-patrol' ? state.cooperative_ready : state.ready;
      b.disabled = !connected || !ready || state.estop || state.mode === 'FAULT' || pending;
    });
    if (state.mode === 'EXPLORING') document.querySelector('[data-command="explore"]').disabled = true;
    $('apply-regions').disabled = !connected || !state.ready || state.regions_locked || pending;
    document.querySelector('[data-command="save"]').disabled = !connected || pending;
    document.querySelector('[data-command="clear-estop"]').disabled = !connected || pending;
  }
  async function send(kind, data) {
    const urgent = kind === 'command' && ['pause', 'estop'].includes(data.command);
    if (pending && !urgent) return;
    const sequence = ++requestSequence;
    if (!urgent) pending = true;
    feedback('正在发送指令…'); updateControls();
    try {
      const result = await fetchJSON('/api/' + kind, {method:'POST', headers:{'Content-Type':'application/json', 'X-Fleet-Run':encodeURI(state.run_dir || '')}, body:JSON.stringify(data)});
      if (sequence === requestSequence) feedback((result.success ? '✓ ' : '⚠ ') + result.message, result.success);
    } catch (error) {
      if (sequence === requestSequence) feedback('指令未确认：' + (error.name === 'AbortError' ? '请求超时，请先查看任务状态再重试' : error.message));
    } finally { if (!urgent) pending = false; updateControls(); }
  }
  const numeric = id => {
    const input = $(id);
    if (!input.checkValidity() || !Number.isFinite(input.valueAsNumber)) throw new Error('请检查数值输入');
    return input.valueAsNumber;
  };
  const guarded = fn => () => { try { fn(); } catch (error) { feedback('输入无效：' + error.message); } };
  document.querySelectorAll('[data-command]').forEach(button => {
    button.addEventListener('click', () => send('command', {command:button.dataset.command}));
  });
  $('set-formation').addEventListener('click', guarded(() => send('formation', {shape:$('shape').value, ...Object.fromEntries(['x','y','yaw','spacing'].map(k => [k,numeric(k)]))})));
  $('apply-regions').addEventListener('click', guarded(() => send('regions', {regions:[...$('regions').children].map(row => [...row.children].map(e => {
    if (!e.checkValidity() || !Number.isFinite(e.valueAsNumber)) throw new Error('区域数值无效');
    return e.valueAsNumber;
  }))})));
  $('start-patrol').addEventListener('click', guarded(() => {
    const plan = {routes:JSON.parse($('routes').value), cycles:numeric('cycles'), dwell:numeric('dwell'), goal_timeout:numeric('goal-timeout')};
    if(state.architecture === 'decentralized') plan.coordinate_frame=state.coordinate_frame;
    send('command', {command:'patrol', argument:JSON.stringify(plan)});
  }));

  const arena = () => state.arena || [-7.5,-7.5,7.5,7.5];
  const point = p => {const a=arena();return [(p[0]-a[0])*960/(a[2]-a[0]), (a[3]-p[1])*960/(a[3]-a[1])];};
  function automaticRegions() {
    const seeds=state.region_seeds||{}, names=Object.keys(seeds).sort(), b=arena();
    if(names.length!==3) return [];
    return names.map(name=>{
      const a=seeds[name]; let polygon=[[b[0],b[1]],[b[2],b[1]],[b[2],b[3]],[b[0],b[3]]];
      names.filter(n=>n!==name).forEach(other=>{
        const q=seeds[other], nx=q[0]-a[0], ny=q[1]-a[1], c=(q[0]*q[0]+q[1]*q[1]-a[0]*a[0]-a[1]*a[1])/2;
        const clipped=[];
        polygon.forEach((p,i)=>{
          const r=polygon[(i+1)%polygon.length], fp=nx*p[0]+ny*p[1]-c, fr=nx*r[0]+ny*r[1]-c;
          if(fp<=0) clipped.push(p);
          if((fp<=0)!==(fr<=0)){const t=fp/(fp-fr);clipped.push([p[0]+t*(r[0]-p[0]),p[1]+t*(r[1]-p[1])]);}
        });
        polygon=clipped;
      });
      return polygon;
    });
  }
  function draw() {
    ctx.fillStyle = '#445160'; ctx.fillRect(0,0,960,960);
    if (image) ctx.drawImage(image,0,0,960,960);
    ctx.strokeStyle='#778a9e33'; ctx.lineWidth=1;
    const bounds=arena();
    for(let x=Math.ceil(bounds[0]);x<bounds[2];x++){const q=point([x,0]);ctx.beginPath();ctx.moveTo(q[0],0);ctx.lineTo(q[0],960);ctx.stroke();}
    for(let y=Math.ceil(bounds[1]);y<bounds[3];y++){const q=point([0,y]);ctx.beginPath();ctx.moveTo(0,q[1]);ctx.lineTo(960,q[1]);ctx.stroke();}
    (state.regions || []).forEach((r,i) => {const a=point([r[0],r[3]]),b=point([r[2],r[1]]);ctx.strokeStyle=colors[i]+(state.region_enabled?'99':'44');ctx.lineWidth=3;ctx.setLineDash([10,10]);ctx.strokeRect(a[0],a[1],b[0]-a[0],b[1]-a[1]);ctx.setLineDash([]);});
    if(state.architecture==='decentralized' && !state.regions?.length) automaticRegions().forEach((polygon,i)=>{
      if(!polygon.length) return;
      ctx.save();ctx.strokeStyle=colors[i]+(state.region_enabled?'99':'44');ctx.fillStyle=colors[i]+'12';ctx.lineWidth=3;ctx.setLineDash([10,10]);ctx.beginPath();
      polygon.forEach((p,j)=>{const q=point(p);j?ctx.lineTo(...q):ctx.moveTo(...q);});ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
    });
    Object.entries(state.robots || {}).forEach(([name,r],i) => {
      ctx.strokeStyle=colors[i];ctx.lineWidth=2;ctx.beginPath();(r.trail||[]).forEach((p,j)=>{const q=point(p);j?ctx.lineTo(...q):ctx.moveTo(...q);});ctx.stroke();
      ctx.save();ctx.setLineDash([9,6]);ctx.lineWidth=3;ctx.beginPath();(r.planned_path||r.plan||[]).forEach((p,j)=>{const q=point(p);j?ctx.lineTo(...q):ctx.moveTo(...q);});ctx.stroke();ctx.restore();
      if(r.goal){const p=point(r.goal);ctx.strokeRect(p[0]-6,p[1]-6,12,12);}
      if(r.pose){const p=point(r.pose);ctx.save();ctx.translate(...p);ctx.rotate(-r.pose[2]);ctx.fillStyle=colors[i];ctx.beginPath();ctx.arc(0,0,14,0,Math.PI*2);ctx.fill();ctx.strokeStyle='#102030';ctx.lineWidth=4;ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(22,0);ctx.stroke();ctx.restore();ctx.fillStyle='#172433';ctx.font='bold 16px sans-serif';ctx.fillText(name,p[0]+18,p[1]-18);}
    });
    (state.formation_slots || []).forEach(p => {const q=point(p);ctx.strokeStyle='#00a78e';ctx.lineWidth=3;ctx.beginPath();ctx.arc(...q,13,0,Math.PI*2);ctx.stroke();});
    ctx.font='14px sans-serif';ctx.fillStyle='#2b4660';ctx.fillText('↑ +Y   → +X    每格 1 m',20,940);
  }
  const metric = (value, unit) => Number.isFinite(value) ? value.toFixed(2) + unit : '未收到';
  function render() {
    $('mode').textContent=({ALIGNING:'待对齐 · 补充观测',DEGRADED:'降级运行',PATROL_COMPLETE:'巡逻完成'}[state.mode]) || labels[state.mode] || state.mode || '正在启动';
    $('message').textContent=state.message || '';
    $('known').textContent='地图已知比例：'+((state.known_fraction||0)*100).toFixed(1)+'%（非真值覆盖率）\n直接观测 '+((state.observed_fraction||0)*100).toFixed(1)+'% + 障碍推断 / 证据保留 '+((state.inferred_obstacle_fraction||0)*100).toFixed(1)+'%\n其中外墙补全 '+((state.boundary_obstacle_fraction||0)*100).toFixed(1)+'%';
    $('planner').textContent='全局规划器：'+(state.planner === 'smac2d' ? 'Smac2D' : 'Navfn A*');
    const generatorName=state.scene?.generator==='mixed-shapes-v2'?'混合形状':state.scene?.generator==='rectangles-v1'?'旧版矩形':'';
    $('scene').textContent='场地 15 × 15 m · 种子 '+(state.scene?.seed || '等待启动')+(generatorName?' · '+generatorName:'');
    $('scope').textContent=state.exploration_unrestricted ? '已解除分区限制，按安全路径就近派车 · '+state.region_release_reason : '探索分工：三个矩形区域；必要时自动跨区补盲';
    if(state.architecture === 'decentralized') {
      const observed=arena();
      $('x').min=observed[0];$('x').max=observed[2];
      $('y').min=observed[1];$('y').max=observed[3];
      $('scene').textContent='本机三车仿真 · 去中心化 · 未知起点';
      $('scope').textContent=state.team_ready ? (state.regions?.length ? '共同相对坐标：手动矩形分区' : '共同相对坐标：自动 Voronoi 分区，优先三车并行探索') : '待对齐；开始探索后限速补充观测，同一时刻一车移动';
      if(state.exploration_unrestricted) $('scope').textContent='共同相对坐标：跨区补盲 · 冲突通道互斥通行';
      $('readiness').textContent=(state.local_ready?'✓ 本车链路就绪':'… 等待本车链路')+' · '+(state.team_ready?'✓ 协同就绪':'… 待对齐')+(state.degraded?' · 降级运行':'')+' · 坐标系 '+(state.coordinate_frame||'尚未建立');
      $('region-help').textContent='默认根据各车对齐后的估计初始位置自动分区。手动调整时，每行填写共同相对坐标中的 xmin、ymin、xmax、ymax；三个矩形须有效且互不重叠。';
      $('patrol-help').textContent='航点使用页面所示的共同相对坐标：[x, y, yaw]。旧世界坐标须先转换或重新输入。未对齐或成员不足的功能保持不可用。';
    }
    $('robots').textContent=Object.entries(state.robots||{}).map(([n,r])=>n+' 探索 '+metric(r.travel,' m')+' / 总行驶 '+metric(r.distance,' m')+' · '+(r.safety?.reason||'运行中')+(r.message?' · '+r.message:'')+(r.goal && r.exploration_target ? '\n  目标可见未知 '+metric(r.exploration_target.visible_unknown_m2,' m²')+' · 路线 '+metric(r.exploration_target.route_length,' m') : '')).join('\n');
    $('errors').textContent=Object.entries(state.formation_errors||{}).map(([n,e])=>n+': '+metric(e.position,' m')+' / '+metric(e.yaw*180/Math.PI,'°')).join(' | ');
    const names={nav2:'Nav2',navigation_pose:'导航位姿',mapping_and_safety:'建图与安全',localization:'定位',initial_free:'起点空闲观测'};
    if(state.architecture !== 'decentralized') $('readiness').textContent=Object.entries(state.readiness||{}).map(([k,v])=>(v?'✓ ':'… ')+(names[k]||k)).join(' · ');
    $('health').replaceChildren(...Object.entries(state.robots||{}).map(([n,r])=>{const tr=document.createElement('tr'),s=r.safety||{};[n,metric(s.scan_age,' s'),metric(s.odom_age,' s'),metric(s.localization_age,' s'),metric(s.nearest_obstacle,' m')].forEach(v=>{const td=document.createElement('td');td.textContent=v;tr.append(td);});if(!s.scan_valid||!s.odom_valid||!s.localization_ok)tr.className='bad';return tr;}));
    const patrol=state.patrol, statuses={preparing:'准备中',running:'运行中',completed:'已完成',cancelled:'已取消',failed:'失败'};
    $('patrol-progress').textContent=patrol ? (statuses[patrol.status]||patrol.status)+' · '+Object.entries(patrol.robots).map(([n,p])=>n+' '+p.completed+'/'+p.total).join(' · ')+(patrol.error?' · '+patrol.error:'') : '尚未提交巡逻任务';
    if(!initialized && (state.regions?.length===3 || Object.keys(state.starts||{}).length===3)){
      const editRegions=state.regions?.length===3 ? state.regions : ['robot1','robot2','robot3'].map(n=>state.starts[n]).map(p=>[p[0]-.9,p[1]-.9,p[0]+.9,p[1]+.9]);
      $('regions').replaceChildren(...editRegions.map(r=>{const d=document.createElement('div');d.className='regions';r.forEach(v=>{const e=document.createElement('input');e.type='number';e.step='.1';e.value=v;d.append(e);});return d;}));
      if(state.starts){$('routes').value=JSON.stringify(Object.fromEntries(Object.entries(state.starts).map(([n,p])=>[n,[[p[0]+(p[0]>0?.6:-.6),p[1],p[2]],p]])));}
      initialized=true;}
    if(state.map_version!==version){version=state.map_version;const requested=version,img=new Image();img.onload=()=>{if(requested===version){image=img;draw();}};img.src='/api/map.png?v='+version;}
    draw(); updateControls();
  }
  async function poll() {
    try { state=await fetchJSON('/api/state',{},3000); connected=true; render(); }
    catch(error){connected=false;$('mode').textContent='连接中断';$('message').textContent='请检查启动终端和运行日志。';updateControls();}
    setTimeout(poll,500);
  }
  updateControls(); poll();
})();
