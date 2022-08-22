import React, {createContext, useEffect, useState} from 'react';
import {w3cwebsocket, w3cwebsocket as W3CWebSocket} from 'websocket';

export class SocketEvents extends EventTarget {
	/**
	 * @param {w3cwebsocket} ws Websocket with events to wrap
	 */
	constructor(ws) {
		super();
		// Set event functions of websocket to dispatchers
		this.setWS(ws);
	}

	/**
	 * @param {w3cwebsocket} ws Websocket with events to wrap
	 */
	setWS(ws) {
		ws.onclose = event => this.dispatch('close', {detail: event});
		ws.onerror = error => this.dispatch('error', {detail: error});
		ws.onmessage = message => {
			this.onMessage(message);
			this.dispatch('message', {detail: {message: message.data}});
		};

		ws.onopen = () => this.dispatch('open');
	}

	/**
	 * Dispatches an event with type string and data.
	 * @param {("message" | "open" | "close" | "error")} ty
	 * @param {Object} ev event data
	 */
	dispatch(ty, ev) {
		this.dispatchEvent(new CustomEvent(ty, ev));
	}

	onMessage(e) {
		const msgJSON = JSON.parse(e.data);
		// eslint-disable-next-line camelcase
		const {event_type, ...data} = msgJSON;
		this.dispatch(event_type, {detail: data});
	}
}

export let Client = new W3CWebSocket(process.env.REACT_APP_BACKEND_SOCKET);
export const Events = new SocketEvents(Client);
// Export const Events = new RealtimeEvents(ClientEvents);

window.Events = Events;

export function restartClient() {
	Client.close();
	Client = null;
	Client = new W3CWebSocket(process.env.REACT_APP_BACKEND_SOCKET);
	Events.setWS(Client);
}

// Log WMS fetches
const winFetch = window.fetch;
const localRegex = /.*localhost:.*/;
window.fetch = (...args) => {
	const {url} = args[0];
	let res;
	if (localRegex.test(url)) {
		res = winFetch(url.replace('localhost', process.env.REACT_APP_SERVER_IP));
	} else {
		const proxurl = `${process.env.REACT_APP_CACHE_URL}/${encodeURIComponent(url)}`;
		res = winFetch(proxurl);
	}

	return res;
};
