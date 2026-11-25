#pragma once
#include <Arduino.h>

const char SUPERVISORY_HTML[] PROGMEM = R"SONAR(
<!doctype html>
<html lang="pt-BR">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Supervisório de sonar</title>
<style>
  :root { color-scheme: dark; --bg:#0a1219; --panel:#12202a; --line:#2a414f; --text:#eff6f8;
    --muted:#afc3ce; --green:#69edbf; --red:#ffb4ae; }
  * { box-sizing:border-box }
  body { margin:0; background:var(--bg); color:var(--text); font:16px system-ui,sans-serif }
  main { max-width:1120px; margin:auto; padding:28px 20px 38px }
  header { display:flex; align-items:baseline; justify-content:space-between; gap:16px; flex-wrap:wrap }
  h1 { font-size:clamp(1.65rem,5vw,2.45rem); margin:0 0 12px; letter-spacing:-.035em }
  h2 { margin:0; font-size:1.05rem }
  #connection { color:var(--muted) }
  .controls { display:flex; flex-wrap:wrap; gap:9px; margin:10px 0 14px }
  [hidden] { display:none !important }
  button { min-height:44px; padding:8px 14px; border:1px solid var(--line); border-radius:9px;
    background:var(--panel); color:var(--text); font:inherit; font-weight:650; cursor:pointer }
  button:focus-visible { outline:2px solid var(--green); outline-offset:2px }
  button:disabled { opacity:.5; cursor:default }
  #start { background:var(--green); color:#092b23; border-color:var(--green) }
  #stop { color:var(--red); border-color:#83545a }
  .stats { display:grid; grid-template-columns:repeat(4,minmax(0,1fr)); gap:10px; margin:0 0 16px }
  .stats div, section { border:1px solid var(--line); background:var(--panel); border-radius:12px; padding:14px }
  dt { color:var(--muted); font-size:.88rem }
  dd { margin:4px 0 0; font-size:1.15rem; font-weight:700; font-variant-numeric:tabular-nums }
  .plots { display:grid; grid-template-columns:1fr 1fr; gap:14px }
  canvas { display:block; width:100%; height:auto; aspect-ratio: 500/310; margin-top:6px }
  .legend { display:flex; flex-wrap:wrap; gap:14px; color:var(--muted); margin-top:6px; font-size:.85rem }
  .legend b { color:var(--green) }
  #notice { min-height:1.5em; margin:0 0 10px; color:var(--red) }
  details { margin-top:16px; border-top:1px solid var(--line); padding:14px 0 }
  table { width:100%; border-collapse:collapse; margin-top:10px; font-variant-numeric:tabular-nums }
  th, td { text-align:right; padding:8px; border-bottom:1px solid var(--line) }
  th:first-child, td:first-child { text-align:left }
  .table-scroll { overflow-x:auto }
  summary { cursor:pointer; width:fit-content; font-weight:650 }
  details p { color:var(--muted); max-width:65ch }
  @media(max-width:700px) { .plots { grid-template-columns:1fr } .stats { grid-template-columns:repeat(2,1fr) } }
</style>
</head>
<body>
<main>
  <header><h1>Supervisório de sonar</h1><span id="connection" role="status">Conectando à ESP32</span></header>
  <div class="controls">
    <button id="start" disabled>Iniciar varredura</button>
    <button id="advance" disabled>Avançar e mapear</button>
    <button id="stop" disabled>Parar</button>
    <button id="clear" disabled>Limpar mapa</button>
  </div>
  <div class="controls" id="usb-controls" hidden>
    <button id="usb">Conectar por USB</button>
  </div>
  <p id="notice" role="alert"></p>
  <p id="advance-status" role="status" hidden></p>
  <dl class="stats">
    <div><dt>Estado</dt><dd id="status">—</dd></div>
    <div><dt>Ângulo comandado</dt><dd id="angle">—</dd></div>
    <div><dt>Última distância</dt><dd id="range">—</dd></div>
    <div><dt>Medições</dt><dd id="progress">—</dd></div>
  </dl>
  <div class="plots">
    <section><h2>Sonar</h2><canvas id="sonar" aria-label="Leituras polares do sonar"></canvas>
      <div class="legend"><span><b>●</b> Eco válido</span><span>Linha indica o ângulo comandado</span></div></section>
    <section><h2>Mapa 2D local</h2><canvas id="map" aria-label="Posição do robô e ecos no plano cartesiano"></canvas>
      <div class="legend"><span><b>●</b> Obstáculo detectado</span><span>▲ Robô na origem</span></div></section>
  </div>
  <details>
    <summary>Leituras da varredura</summary>
    <p id="empty-readings">Aguardando medições.</p>
    <div class="table-scroll">
      <table id="readings-table" aria-label="Medições da varredura" hidden>
        <thead><tr><th scope="col">Ângulo</th><th scope="col">Distância</th><th scope="col">X lateral</th><th scope="col">Y à frente</th></tr></thead>
        <tbody id="readings"></tbody>
      </table>
    </div>
  </details>
  <details>
    <summary>Ajustar a montagem do sensor</summary>
    <p>Centralize o SG90 em 90° e alinhe o sensor à frente do robô.</p>
    <button id="center" disabled>Centralizar servo</button>
  </details>
  <details>
    <summary>Teste dos motores em malha aberta</summary>
    <p>Avanço por 1 segundo com PWM 175 à esquerda e 180 à direita. O teste limpa o mapa anterior e fica bloqueado durante a varredura.</p>
    <button id="motor" disabled>Testar motores</button>
  </details>
</main>
<script>
'use strict';
const $ = id => document.getElementById(id);
const labels = {idle:'Pronto', checking:'Checando a frente', advancing:'Avançando',
  settling:'Parado, preparando varredura', homing:'Indo para o extremo esquerdo',
  returning:'Voltando ao centro', moving:'Movendo para o próximo ponto', running:'Medindo', stopped:'Interrompida', done:'Concluída', error:'Falha no servo'};
const advanceLabels = {
  noEcho:'Sem eco frontal válido. Avanço interrompido por segurança.',
  timeLimit:'Avanço encerrado pelo limite de tempo ou de atualização do sensor.',
  motorFailure:'Os motores não iniciaram o avanço.'
};
let state = null, online = false, pending = false, version = 0;
const usbMode = ['localhost','127.0.0.1'].includes(location.hostname);
const encoder = new TextEncoder();
let usb = null, usbConnecting = false, requestId = 0;
const origin = {x:250, y:267}, radius = 214;

function coordinates(angle, cm) {
  const theta = angle * Math.PI / 180;
  return [-cm * Math.cos(theta), cm * Math.sin(theta)];
}
function project(angle, cm, maximum) {
  const [x,y] = coordinates(angle,cm);
  return [origin.x + x / maximum * radius, origin.y - y / maximum * radius];
}
function formatDistance(cm) {
  const rounded = Math.round(cm * 10) / 10;
  return rounded.toFixed(1).replace('.',',')+' cm';
}
function renderReadings(points) {
  $('empty-readings').hidden = points.length > 0;
  $('readings-table').hidden = points.length === 0;
  // Os valores são ângulos e distâncias numéricos retornados pelo firmware.
  $('readings').innerHTML = points.map(([angle,distance]) => {
    if (!Number.isFinite(angle)) return '';
    const valid = Number.isFinite(distance);
    const [x,y] = valid ? coordinates(angle,distance) : [0,0];
    return '<tr><td>'+angle+'°</td><td>'+(valid ? formatDistance(distance) : 'Sem eco válido')
      +'</td><td>'+(valid ? formatDistance(x) : '—')+'</td><td>'
      +(valid ? formatDistance(y) : '—')+'</td></tr>';
  }).join('');
}
function line(ctx, x1, y1, x2, y2, color) {
  ctx.strokeStyle = color;
  ctx.beginPath(); ctx.moveTo(x1,y1); ctx.lineTo(x2,y2); ctx.stroke();
}
function draw(id, polar) {
  const canvas = $(id), dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(canvas.clientWidth * dpr));
  const height = Math.max(1, Math.round(canvas.clientHeight * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width; canvas.height = height;
  }
  const ctx = canvas.getContext('2d'), maximum = state?.maxCm ?? 400;
  const ink = '#afc3ce', grid = '#304a59';
  ctx.setTransform(width / 500,0,0,height / 310,0,0);
  ctx.clearRect(0,0,500,310);
  ctx.fillStyle = ink;
  ctx.font = '13px system-ui';
  ctx.textAlign = 'center';

  if (polar) {
    for (let fraction = 0.25; fraction <= 1; fraction += 0.25) {
      ctx.strokeStyle = grid;
      ctx.beginPath();
      ctx.arc(origin.x,origin.y,radius*fraction,Math.PI,Math.PI*2);
      ctx.stroke();
      ctx.fillText(Math.round(maximum*fraction) + ' cm',origin.x+26,origin.y-radius*fraction+16);
    }
    for (const angle of [0,30,60,90,120,150,180]) {
      const [x,y] = project(angle,maximum,maximum);
      const [lx,ly] = project(angle,maximum*1.085,maximum);
      line(ctx,origin.x,origin.y,x,y,grid);
      ctx.fillText(angle+'°',lx,ly+5);
    }
    if (state) {
      const [x,y] = project(state.angle,maximum,maximum);
      line(ctx,origin.x,origin.y,x,y,'#53c8f8');
    }
  } else {
    for (let fraction = -1; fraction <= 1; fraction += 0.25) {
      const x = origin.x + fraction * radius;
      line(ctx,x,origin.y-radius,x,origin.y,grid);
      if (fraction === -1 || fraction === 0 || fraction === 1) {
        ctx.fillText(Math.round(fraction*maximum),x,origin.y+29);
      }
    }
    for (let fraction = 0; fraction <= 1; fraction += 0.25) {
      const y = origin.y - fraction * radius;
      line(ctx,origin.x-radius,y,origin.x+radius,y,grid);
      if (fraction) ctx.fillText(Math.round(fraction*maximum),17,y-4);
    }
    ctx.fillText('Frente (cm)',origin.x,28);
    ctx.fillStyle = '#e0eefa';
    ctx.beginPath(); ctx.moveTo(origin.x,origin.y-10);
    ctx.lineTo(origin.x+7,origin.y+7); ctx.lineTo(origin.x-7,origin.y+7); ctx.fill();
  }

  ctx.fillStyle = '#69edbf';
  for (const [angle, distance] of state?.points ?? []) {
    if (distance === null) continue;
    const [x,y] = project(angle,distance,maximum);
    ctx.beginPath(); ctx.arc(x,y,4,0,2*Math.PI); ctx.fill();
  }
}
function render() {
  $('connection').textContent = online
    ? (usbMode ? 'ESP32 conectada por USB' : 'ESP32 conectada')
    : (usb ? 'Aguardando resposta da ESP32' : 'Sem conexão com a ESP32');
  $('usb-controls').hidden = !usbMode;
  $('usb').disabled = usbConnecting;
  $('usb').textContent = usb ? 'Desconectar USB' : 'Conectar por USB';
  const phase = state?.state;
  const busy = ['running','checking','advancing','settling','homing','moving','returning'].includes(phase);
  $('status').textContent = !busy && state?.motorBusy ? 'Motores / aguardando parada' : (labels[phase] ?? '—');
  const front = Number.isFinite(state?.frontCm) ? formatDistance(state.frontCm) : null;
  const result = state?.advanceEnd === 'obstacle' && front
    ? 'Parou por obstáculo a '+front+'.' : advanceLabels[state?.advanceEnd];
  $('advance-status').textContent = result ?? (phase === 'advancing' && front ? 'Distância frontal: '+front : '');
  $('advance-status').hidden = !$('advance-status').textContent;
  if (phase === 'done' && Number.isFinite(state.scanEchoes)) {
    if (state.scanEchoes === 0) $('status').textContent += ' · sem pulsos no ECHO (D19)';
    else if (state.points.every(([,cm]) => cm === null)) {
      $('status').textContent += ' · ecos sem concordância';
    }
  }
  $('angle').textContent = state ? state.angle + '°' : '—';
  const reading = state?.points.at(-1);
  $('range').textContent = !reading ? '—' :
    (reading[1] === null ? 'Sem eco válido' : formatDistance(reading[1]))+' · '+reading[0]+'°';
  $('progress').textContent = state ? state.points.length+' / '+state.total : '—';
  const servoBlocked = !online || pending || busy || state?.motorBusy || state?.servoReady === false;
  $('start').disabled = servoBlocked;
  $('advance').disabled = servoBlocked || !state?.motorReady;
  $('center').disabled = servoBlocked;
  $('stop').disabled = !online;
  $('clear').disabled = !online || pending || busy || !state?.points.length;
  $('motor').disabled = !online || pending || busy || state?.motorBusy || !state?.motorReady;
  renderReadings(state?.points ?? []);
  draw('sonar',true);
  draw('map',false);
}
function usbRequest(action) {
  const link = usb;
  if (!link) return Promise.reject(new Error('Conecte a ESP32 por USB.'));
  const id = ++requestId;
  return new Promise((resolve,reject) => {
    const finish = (error,value) => {
      clearTimeout(timeout);
      link.requests.delete(id);
      if (error) reject(error); else resolve(value);
    };
    const timeout = setTimeout(() => finish(new Error('A ESP32 não respondeu por USB.')),2200);
    link.requests.set(id,finish);
    link.writer.write(encoder.encode('@'+id+' '+action+'\n')).catch(error => finish(error));
  });
}
function parseUsbLine(link,line) {
  const match = /^@(\d+) (\d+) (.+)$/.exec(line.trim());
  if (!match) return;
  const finish = link.requests.get(Number(match[1]));
  if (!finish) return;
  try {
    if (match[2] !== '200') throw new Error(match[3]);
    finish(null,JSON.parse(match[3]));
  } catch (error) { finish(error); }
}
async function closeUsb(link) {
  if (link.closing) return;
  link.closing = true;
  if (usb === link) {
    usb = null;
    online = false;
    state = null;
    pending = false;
    ++version;
    render();
  }
  for (const finish of link.requests.values()) finish(new Error('USB desconectado.'));
  try { await link.reader.cancel(); } catch {}
  await link.reading;
  link.writer.releaseLock();
  try { await link.port.close(); } catch {}
}
async function readUsb(link) {
  const decoder = new TextDecoder();
  let buffer = '';
  try {
    while (!link.closing) {
      const {value,done} = await link.reader.read();
      if (done) break;
      buffer += decoder.decode(value,{stream:true});
      const lines = buffer.split('\n');
      buffer = lines.pop();
      for (const line of lines) parseUsbLine(link,line);
      if (buffer.length > 4096) buffer = '';
    }
  } catch {
    if (!link.closing) $('notice').textContent = 'A conexão USB foi interrompida.';
  } finally {
    link.reader.releaseLock();
    if (!link.closing) void closeUsb(link);
  }
}
async function toggleUsb() {
  if (usb) { await closeUsb(usb); return; }
  if (!navigator.serial) {
    $('notice').textContent = 'Abra este painel no Microsoft Edge ou no Google Chrome.';
    return;
  }
  usbConnecting = true;
  $('notice').textContent = '';
  render();
  try {
    const port = await navigator.serial.requestPort();
    await port.open({baudRate:115200});
    const link = {port,reader:port.readable.getReader(),writer:port.writable.getWriter(),
                  requests:new Map(),closing:false};
    usb = link;
    ++version;
    link.reading = readUsb(link);
  } catch (error) {
    $('notice').textContent = error.name === 'NotFoundError'
      ? 'Selecione a porta da ESP32 para conectar.'
      : 'Não foi possível abrir a porta. Feche o Monitor Serial do Arduino e tente de novo.';
  } finally {
    usbConnecting = false;
    render();
  }
}
async function request(action) {
  if (usbMode) return usbRequest(action);
  const options = {cache:'no-store',signal:AbortSignal.timeout(3000)};
  if (action !== 'state') {
    options.method = 'POST';
    options.headers = {'X-Robot-Command':'local'};
  }
  const url = action === 'state' ? '/state' : '/command?action='+action;
  const response = await fetch(url,options);
  if (!response.ok) throw new Error(await response.text());
  return response.json();
}
async function refresh() {
  const observedVersion = version;
  try {
    if (!pending && (!usbMode || usb)) {
      const next = await request('state');
      if (observedVersion === version && !pending) { state = next; online = true; }
    }
  } catch {
    if (observedVersion === version && !pending) online = false;
  } finally {
    render();
    setTimeout(refresh,350);
  }
}
async function send(action) {
  if (!online || (pending && action !== 'stop')) return;
  if (action === 'motor' && !confirm('Teste de avanço de 1 segundo. O caminho está livre?')) return;
  if (action === 'advance' && !confirm('O robô vai avançar por até 5 segundos. A área e o cabo estão livres?')) return;
  const issued = ++version;
  pending = true;
  $('notice').textContent = '';
  render();
  try {
    const next = await request(action);
    if (issued === version) state = next;
  } catch (error) {
    if (issued === version) $('notice').textContent = error.message;
  } finally {
    if (issued === version) { pending = false; render(); }
  }
}
for (const action of ['start','advance','stop','clear','center','motor']) {
  $(action).addEventListener('click',() => send(action));
}
$('usb').addEventListener('click',toggleUsb);
window.addEventListener('resize',render);
refresh();
</script>
</body>
</html>
)SONAR";
