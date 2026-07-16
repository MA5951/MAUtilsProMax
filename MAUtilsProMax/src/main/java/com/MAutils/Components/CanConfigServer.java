package com.MAutils.Components;

import java.io.IOException;
import java.io.InputStream;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;

// ------------------------------
// CanConfigServer — HTTP server 
// ------------------------------
public final class CanConfigServer {
  private static HttpServer server;
  private static ExecutorService httpPool;

  public static void start(int port) {
    CanService can = new CanServiceImpl();
    if (server != null) return;
    try {
      server = HttpServer.create(new InetSocketAddress(port), 0);
      httpPool = Executors.newCachedThreadPool(r -> {
        Thread t = new Thread(r, "CanConfig-HTTP");
        t.setDaemon(true);
        return t;
      });
      server.setExecutor(httpPool);

      server.createContext("/", ex -> {
        if (!ex.getRequestMethod().equals("GET")) { sendStatus(ex, 405); return; }
        byte[] html = StaticIndexHTML.CONTENT.getBytes(StandardCharsets.UTF_8);
        ex.getResponseHeaders().set("Content-Type", "text/html; charset=utf-8");
        ex.sendResponseHeaders(200, html.length);
        try (var out = ex.getResponseBody()) { out.write(html); }
      });

      server.createContext("/api/devices", ex -> {
        try {
          if (ex.getRequestMethod().equals("GET")) {
            var devices = can.enumerate();
            sendJson(ex, 200, Json.buildDevices(devices));
          } else if (ex.getRequestMethod().equals("OPTIONS")) {
            sendCORS(ex);
          } else sendStatus(ex, 405);
        } catch (Exception e) { sendError(ex, e); }
      });

      server.createContext("/api/devices/set-id", ex -> {
        try {
          String method = ex.getRequestMethod();
          if (method.equals("PATCH") || method.equals("POST")) {
            Map<String,String> kv = Json.parseFlat(readBody(ex));
            boolean ok = can.setId(kv.get("type"), Integer.parseInt(kv.get("id")), Integer.parseInt(kv.get("newId")));
            sendStatus(ex, ok ? 204 : 400);
          } else if (method.equals("OPTIONS")) sendCORS(ex);
          else sendStatus(ex, 405);
        } catch (Exception e) { sendError(ex, e); }
      });

      server.createContext("/api/devices/identify", ex -> {
        try {
          String method = ex.getRequestMethod();
          if (method.equals("POST")) {
            Map<String,String> kv = Json.parseFlat(readBody(ex));
            boolean ok = can.identify(kv.get("type"), Integer.parseInt(kv.get("id")));
            sendStatus(ex, ok ? 204 : 400);
          } else if (method.equals("OPTIONS")) sendCORS(ex);
          else sendStatus(ex, 405);
        } catch (Exception e) { sendError(ex, e); }
      });

      server.createContext("/api/devices/set-name", ex -> {
        try {
          String method = ex.getRequestMethod();
          if (method.equals("POST") || method.equals("PATCH")) {
            Map<String,String> kv = Json.parseFlat(readBody(ex));
            String name = kv.getOrDefault("name", "").trim();
            boolean ok = !name.isEmpty() && can.setName(kv.get("type"), Integer.parseInt(kv.get("id")), name);
            sendStatus(ex, ok ? 204 : 400);
          } else if (method.equals("OPTIONS")) sendCORS(ex);
          else sendStatus(ex, 405);
        } catch (Exception e) { sendError(ex, e); }
      });

      server.start();
      System.out.println("[CanConfig] HTTP started on :" + port);
    } catch (IOException ioe) {
      ioe.printStackTrace();
    }
  }

  public static void stop() {
    if (server != null) server.stop(0);
    if (httpPool != null) httpPool.shutdownNow();
    server = null;
    httpPool = null;
  }

  private static String readBody(HttpExchange ex) throws IOException {
    try (InputStream in = ex.getRequestBody()) {
      return new String(in.readAllBytes(), StandardCharsets.UTF_8);
    }
  }

  private static void sendJson(HttpExchange ex, int code, String json) throws IOException {
    var h = ex.getResponseHeaders();
    h.set("Content-Type", "application/json; charset=utf-8");
    h.set("Cache-Control", "no-store");
    h.set("Access-Control-Allow-Origin", "*");
    byte[] b = json.getBytes(StandardCharsets.UTF_8);
    ex.sendResponseHeaders(code, b.length);
    try (var out = ex.getResponseBody()) { out.write(b); }
  }

  private static void sendStatus(HttpExchange ex, int code) throws IOException {
    ex.getResponseHeaders().set("Access-Control-Allow-Origin", "*");
    if (code == 204) ex.sendResponseHeaders(code, -1);
    else { ex.sendResponseHeaders(code, 0); try (var out = ex.getResponseBody()) {} }
  }

  private static void sendCORS(HttpExchange ex) throws IOException {
    var h = ex.getResponseHeaders();
    h.set("Access-Control-Allow-Origin", "*");
    h.set("Access-Control-Allow-Methods", "GET,POST,PATCH,OPTIONS");
    h.set("Access-Control-Allow-Headers", "Content-Type");
    ex.sendResponseHeaders(204, -1);
  }

  private static void sendError(HttpExchange ex, Exception e) throws IOException {
    e.printStackTrace();
    String msg = Json.escape("Error: " + e.getMessage());
    sendJson(ex, 500, "{\"error\":\"" + msg + "\"}");
  }
}

// ------------------------------
// CanService — interface
// ------------------------------
interface CanService {
  record Device(String type, int id, String name, String fw, String status, double ledDistance) {}
  List<Device> enumerate();
  boolean setId(String type, int id, int newId);
  boolean identify(String type, int id);
  boolean setLedDistance(String type, int id, double ledDistance); // unused here
  boolean setName(String type, int id, String newName);
}

// ---------------------------------
// CanServiceImpl — REAL hardware (WPILib CAN)
// ---------------------------------
class CanServiceImpl implements CanService {
  // Must match firmware:
  private static final int MANUFACTURER = 8;  // Team Use
  private static final int DEVICE_TYPE  = 5;  // Distance Sensor

  private static final int API_STATUS     = 0x300;
  private static final int API_RTR_NAME   = 0x310;
  private static final int API_SET_NAME   = 0x311;
  private static final int API_SET_ID     = 0x312;
  private static final int API_IDENTIFY   = 0x313;

  @Override
  public List<Device> enumerate() {
    List<Device> out = new ArrayList<>();
    CANData status = new CANData();

    for (int id = 0; id <= 63; id++) {
      try (CAN can = new CAN(id, MANUFACTURER, DEVICE_TYPE)) {
        // Probe: do we have status packets?
        if (!can.readPacketTimeout(API_STATUS, 5, status)) continue;

        int fwMaj = status.data[4] & 0xFF;
        int fwMin = status.data[5] & 0xFF;
        int fwPat = status.data[6] & 0xFF;

        String fw = fwMaj + "." + fwMin + "." + fwPat;

        // Ask for name via RTR
        String name = readName(can);
        if (name.isBlank()) name = "MACam";

        out.add(new Device("MACam", id, name, fw, "OK", 0.0));
      } catch (Exception ignored) {
        // ignore ids that fail init
      }
    }
    return out;
  }

  private String readName(CAN can) {
    try {
      can.writeRTRFrame(8, API_RTR_NAME);
      CANData nameFrame = new CANData();
      if (!can.readPacketTimeout(API_RTR_NAME, 10, nameFrame)) return "";
      int len = Math.min(8, nameFrame.length);
      byte[] b = new byte[len];
      System.arraycopy(nameFrame.data, 0, b, 0, len);
      // trim trailing zeros/spaces
      int end = len;
      while (end > 0 && (b[end - 1] == 0 || b[end - 1] == ' ')) end--;
      return new String(b, 0, end, StandardCharsets.US_ASCII).trim();
    } catch (Exception e) {
      return "";
    }
  }

  @Override
  public boolean setId(String type, int id, int newId) {
    newId &= 0x3F;
    try (CAN can = new CAN(id, MANUFACTURER, DEVICE_TYPE)) {
      can.writePacket(new byte[] { (byte)newId }, API_SET_ID);
      return true;
    } catch (Exception e) {
      return false;
    }
  }

  @Override
  public boolean identify(String type, int id) {
    try (CAN can = new CAN(id, MANUFACTURER, DEVICE_TYPE)) {
      can.writePacket(new byte[] { 2 }, API_IDENTIFY); // 2 seconds
      return true;
    } catch (Exception e) {
      return false;
    }
  }

  @Override
  public boolean setLedDistance(String type, int id, double ledDistance) {
    // Not used for this sensor; keep endpoint compatibility
    return false;
  }

  @Override
  public boolean setName(String type, int id, String newName) {
    try (CAN can = new CAN(id, MANUFACTURER, DEVICE_TYPE)) {
      byte[] data = new byte[8];
      byte[] src = newName.getBytes(StandardCharsets.US_ASCII);
      int n = Math.min(8, src.length);
      System.arraycopy(src, 0, data, 0, n);
      can.writePacket(data, API_SET_NAME);
      return true;
    } catch (Exception e) {
      return false;
    }
  }
}

// ---------------------------------
// CanServiceSim — desktop simulator
// ---------------------------------
class CanServiceSim implements CanService {
  private final List<Device> fake = new ArrayList<>(List.of(
    new Device("MACam", 20, "Front", "0.3.0", "OK", 0.0),
    new Device("MACam", 21, "Rear",  "0.3.0", "OK", 0.0)
  ));

  @Override public List<Device> enumerate() { return new ArrayList<>(fake); }

  @Override public boolean setId(String type, int id, int newId) {
    for (int i=0;i<fake.size();i++) {
      var d = fake.get(i);
      if (d.id()==id) { fake.set(i, new Device(d.type(), newId, d.name(), d.fw(), d.status(), d.ledDistance())); return true; }
    }
    return false;
  }

  @Override public boolean identify(String type, int id) { return true; }
  @Override public boolean setLedDistance(String type, int id, double ledDistance) { return false; }

  @Override public boolean setName(String type, int id, String newName) {
    for (int i=0;i<fake.size();i++) {
      var d = fake.get(i);
      if (d.id()==id) { fake.set(i, new Device(d.type(), d.id(), newName, d.fw(), d.status(), d.ledDistance())); return true; }
    }
    return false;
  }
}

// ------------------------------
// StaticIndexHTML — UI (fixed IDs)
// ------------------------------
final class StaticIndexHTML {
  static final String CONTENT = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>MA CAN Config · :5812</title>
  <style>
    :root { --bg:#0f0f12; --fg:#e6e6e6; --muted:#9aa0a6; --card:#17171b; --red:#ed1c24; }
    body { margin:0; background:var(--bg); color:var(--fg); font:14px/1.4 system-ui,Segoe UI,Roboto,Ubuntu; }
    header { padding:16px 20px; border-bottom:1px solid #222; display:flex; gap:12px; align-items:center; }
    h1 { margin:0; font-size:18px; font-weight:600; letter-spacing:.3px; }
    .pill { padding:4px 8px; border-radius:999px; background:#222; color:var(--muted); font-size:12px; }
    main { padding:20px; max-width:1000px; margin:auto; }
    table { width:100%; border-collapse:separate; border-spacing:0 8px; }
    th { text-align:left; color:var(--muted); font-weight:600; font-size:12px; }
    td, th { padding:10px 12px; }
    tr.data { background:var(--card); border-radius:12px; }
    tr.data td:first-child { border-top-left-radius:12px; border-bottom-left-radius:12px; }
    tr.data td:last-child { border-top-right-radius:12px; border-bottom-right-radius:12px; }
    button { background:#2a2a31; color:var(--fg); border:1px solid #333; padding:8px 10px; border-radius:10px; cursor:pointer; }
    button.primary { background:var(--red); border-color:#a5161c; }
    .row { display:flex; gap:8px; align-items:center; flex-wrap:wrap; }
    input[type=number], input[type=text] { background:#111; color:var(--fg); border:1px solid #333; padding:6px 8px; border-radius:8px; }
    footer { color:var(--muted); padding:24px 20px; text-align:center; }
  </style>
</head>
<body>
  <header>
    <h1>MA CAN Config</h1>
    <span class="pill">roboRIO :5951</span>
  </header>
  <main>
    <section>
      <div style="margin:14px 0">
        <button onclick="loadDevices()" class="primary">Refresh</button>
      </div>
      <table id="tbl">
        <thead>
          <tr><th>Type</th><th>CAN ID</th><th>Name</th><th>FW</th><th>Status</th><th>Actions</th></tr>
        </thead>
        <tbody id="rows"></tbody>
      </table>
    </section>
  </main>
  <footer>FRC 5951</footer>

<script>
async function loadDevices(){
  const r = await fetch('/api/devices');
  const data = await r.json();
  const tbody = document.getElementById('rows');
  tbody.innerHTML = '';

  const devs = (data.devices||[]);
  if(devs.length === 0){
    const tr = document.createElement('tr'); tr.className='data';
    tr.innerHTML = `<td colspan="6" style="color:var(--muted)">No devices found. Check CAN wiring + 1Mbps and that firmware is sending API 0x300 status.</td>`;
    tbody.appendChild(tr);
    return;
  }

  devs.forEach(d => {
    const tr = document.createElement('tr'); tr.className='data';

    tr.innerHTML = `
      <td>${d.type}</td>
      <td><input class="idBox" type="number" min="0" max="63" value="${d.id}" style="width:90px"/></td>
      <td><input class="nameBox" type="text" value="${d.name}" style="width:160px"/></td>
      <td>${d.fw}</td>
      <td>${d.status}</td>
      <td>
        <div class="row">
          <button class="identifyBtn">Identify</button>
          <button class="setIdBtn primary">Set ID</button>
          <button class="setNameBtn">Save Name</button>
        </div>
      </td>`;

    tr.querySelector('.identifyBtn').onclick = () => identify(d.type, d.id);
    tr.querySelector('.setIdBtn').onclick = async () => {
      const newId = parseInt(tr.querySelector('.idBox').value, 10);
      if(Number.isNaN(newId)) return alert('Invalid ID');
      const rr = await fetch('/api/devices/set-id', {method:'PATCH', headers:{'Content-Type':'application/json'}, body:JSON.stringify({type:d.type,id:d.id,newId})});
      if(rr.status===204) loadDevices(); else alert('Failed to set ID');
    };
    tr.querySelector('.setNameBtn').onclick = async () => {
      const name = (tr.querySelector('.nameBox').value||'').trim();
      if(!name) return alert('Enter a name');
      const rr = await fetch('/api/devices/set-name', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({type:d.type,id:d.id,name})});
      if(rr.status!==204) alert('Failed to set name');
    };

    tbody.appendChild(tr);
  });
}

async function identify(type,id){
  await fetch('/api/devices/identify', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({type,id})});
}

loadDevices();
</script>
</body>
</html>
""";
}

// ------------------------------
// Json — tiny helpers
// ------------------------------
final class Json {
  static String escape(String s) { return s.replace("\\", "\\\\").replace("\"", "\\\""); }

  static String buildDevices(List<CanService.Device> ds) {
    if (ds == null) ds = List.of();
    StringBuilder sb = new StringBuilder();
    sb.append("{\"devices\":[");
    for (int i=0;i<ds.size();i++) {
      var d = ds.get(i);
      if (i>0) sb.append(',');
      sb.append("{\"type\":\"").append(escape(d.type())).append("\",")
        .append("\"id\":").append(d.id()).append(',')
        .append("\"name\":\"").append(escape(d.name())).append("\",")
        .append("\"fw\":\"").append(escape(d.fw())).append("\",")
        .append("\"status\":\"").append(escape(d.status())).append("\",")
        .append("\"ledDistance\":").append(Double.toString(d.ledDistance()))
        .append("}");
    }
    sb.append("]}");
    return sb.toString();
  }

  static Map<String,String> parseFlat(String json) {
    Map<String,String> m = new HashMap<>();
    if (json == null) return m;
    String body = json.trim();
    if (body.startsWith("{")) body = body.substring(1);
    if (body.endsWith("}")) body = body.substring(0, body.length()-1);
    if (body.isEmpty()) return m;
    for (String part : body.split(",")) {
      String[] kv = part.split(":", 2);
      if (kv.length != 2) continue;
      String k = kv[0].trim();
      String v = kv[1].trim();
      if (k.startsWith("\"") && k.endsWith("\"")) k = k.substring(1, k.length()-1);
      if (v.startsWith("\"") && v.endsWith("\"")) v = v.substring(1, v.length()-1);
      m.put(k, v);
    }
    return m;
  }
}
