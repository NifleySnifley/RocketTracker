import React, {useEffect, useState} from 'react';
import {w3cwebsocket as W3CWebSocket} from 'websocket';

const client = new W3CWebSocket('ws://localhost:9002');

export default function WSTerminal(_props) {
	const [data, setData] = useState('');
	useEffect(() => {
		client.onopen = () => {
			console.log('WebSocket Client Connected');
		};

		client.onmessage = message => {
			setData(data + `${message.data}\n`);
		};
	});

	return (
		<code style={{whiteSpace: 'pre-wrap', overflow: 'scroll'}}>{data}</code>
	);
}
