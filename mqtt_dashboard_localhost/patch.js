const fs = require('fs');
const file = 'server.js';
let content = fs.readFileSync(file, 'utf8');

const oldCode1 = `      const routePath = (event.payload && Array.isArray(event.payload.route_path))
        ? event.payload.route_path.join(" -> ")
        : "";
      // Hop otomatis dari route_path, fallback ke event.hops
      const hopDisplay = (event.payload && Array.isArray(event.payload.route_path) && event.payload.route_path.length > 0)
        ? (event.payload.route_path.length - 1)
        : (event.hops ?? "");`;

const newCode = `      let routePathArr = [];
      if (event.payload && Array.isArray(event.payload.route_path)) {
        routePathArr = event.payload.route_path;
      } else if (state.latestByNode && state.latestByNode[event.node]) {
        const latestEvt = state.latestByNode[event.node];
        if (latestEvt && latestEvt.payload && Array.isArray(latestEvt.payload.route_path)) {
          routePathArr = latestEvt.payload.route_path;
        }
      }

      const routePath = routePathArr.length > 0 ? routePathArr.join(" -> ") : "";
      // Hop otomatis dari route_path, fallback ke event.hops
      const hopDisplay = routePathArr.length > 0 ? (routePathArr.length - 1) : (event.hops ?? "");`;

let parts = content.split(oldCode1);
console.log('Found ' + (parts.length - 1) + ' matches');
content = parts.join(newCode);
fs.writeFileSync(file, content);
