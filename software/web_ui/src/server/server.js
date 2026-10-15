import express from "express";
import { createServer } from "http";
import { Server } from "socket.io";

import { handler } from "../../build/handler.js";
import { resourceMonitor } from "./scripts/resourceMonitorSocket.js";
import { cameraSocket } from "./scripts/camerasSocket.js";
import { hostname, networkInterfaces } from "os";

const port = 3000;
const app = express();
const server = createServer(app);

const io = new Server(server);

let clientCount = 0;

io.on("connection", (socket) => {
  clientCount++;
  console.log(`Client connected. ${clientCount} clients connected`);

  // functions that act as websocket end points go here

  socket.on("disconnect", () => {
    clientCount--;
    console.log(`Client disconnected. ${clientCount} clients connected`);
  });

  cameraSocket(socket, io);
});

resourceMonitor(io);

app.use(handler);

server.listen(port, () => {

  const nets = networkInterfaces();
  let results = "";

  Object.entries(nets).forEach(([name, net]) => {
    if (!net) return;
    for (const n of net) {
      // Skip over non-IPv4 and internal addresses
      // 'IPv4' is in Node <= 17, from 18 it's a number 4 or 6
      const familyV4Value = typeof n.family === 'string' ? 'IPv4' : 4
      if (n.family === familyV4Value && !n.internal) {
        results += `http://${n.address}:${port}\n`;
      }
    }
  });
  console.log(
    `Beep boop! Server is running, try connecting on:\x1b[34m\n${results}http://${hostname()}:${port}\x1b[0m`,
  );
});
