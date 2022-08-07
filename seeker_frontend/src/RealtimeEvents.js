import React, {createContext, useEffect, useState} from 'react';
import {w3cwebsocket, w3cwebsocket as W3CWebSocket} from 'websocket';

export class RealtimeEvents extends EventTarget {
	/**
	 * @param {SocketEvents} se {@link SocketEvents} connected to websocket that will be used for events
	 */
	constructor(se) {
		super();
		se.addEventListener('message', this.onMessage);
	}

	/**
	 * Dispatches an event with name and data.
	 * At the core, it just creates a {@link CustomEvent} and dispatches it.
	 * @param {("debug")} iden
	 * @param {Object} data
	 */
	dispatch(iden, data) {
		this.dispatchEvent(new CustomEvent(iden, data));
	}

	/**
	 * Message handler for the websocket client to transform websocket messages into dispatchable events
	 * @param {CustomEvent} e
	 */
	onMessage(e) {
		console.log(e.detail.message.data);
	}
}

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
		ws.onclose = event => this.dispatch('close', event);
		ws.onerror = error => this.dispatch('error', error);
		ws.onmessage = message => this.dispatch('message', {detail: {message}});
		ws.onopen = () => this.dispatch('open');
	}

	/**
	 * Dispatches an event with type string and data.
	 * @param {("message" | "open" | "close" | "error")} ty
	 * @param {Object} ev event data
	 */
	dispatch(ty, ev) {
		this.dispatchEvent(ev ? new CustomEvent(ty, ev) : new Event(ty));
	}
}

// FIXME Sometimes it misses a message and the two apparently come in at the same time, maybe its a backend thing?

export let Client = new W3CWebSocket(process.env.REACT_APP_BACKEND_SOCKET);
export const ClientEvents = new SocketEvents(Client);
export const Events = new RealtimeEvents(ClientEvents);

export function restartClient() {
	Client.close();
	Client = null;
	Client = new W3CWebSocket(process.env.REACT_APP_BACKEND_SOCKET);
	ClientEvents.setWS(Client);
}

