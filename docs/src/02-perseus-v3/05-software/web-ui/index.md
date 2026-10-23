---
title: Web UI
---

# Web UI

The Web UI is a web application which is almost entirely independent of the rovers core software. The UI is a [SvelteKit](https://svelte.dev/) + [Vite](https://vite.dev/) app with an injected Node.js server.

The purpose of this project is to provide a custom user interface for development, testing and operation of the rover. The Web UI is designed to be modular and extensible, meaning developers only need to write essential code as the core Web UI should handle most boilerplate connection code and provide strong design patterns.

This section of the documentation will provide an overview of the Web UI, its architecture and how to use it. Details for contributing and maintaining the Web UI can be found in the [Development section](./development.md). A manual for using the Web UI and widget documentation can be found in the [User Guide](./user-guide.md) ([WebRTC Video](./webrtc-video.md) has been given its own page due to its complexity and importance). For further reading there is also a [tutorial](./../../04-tutorials/web-ui-widget.md) which will walk you through the process of creating a new widget and adding it to the Web UI.
