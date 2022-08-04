# Seeker Backend

The seeker backend is an interface between the GUI you see as a user and the rocket tracker. Itprovides communications with the rocket tracker (or a simulation!) and communicates with the frontend (GUI) in real-time using websockets.

## WIP: TODOs

- [ ] Implement framework for a monolithic simulator that can support multiple adapters for simulating different aspects of the software's input
- [ ] Implement real adapters for the seeker hardware
- [ ] Add websocket communications to stream data from multiple adapters and the tracker to the frontend in a efficient and extensible way. (JSON?)
- [ ] Implement a standardized API for websocket requests so the frontend can request data and push events to/from the backend
- [ ] Add structures with JSON schema for websocket IPC
  