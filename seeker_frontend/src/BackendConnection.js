import React, {useEffect, useState} from 'react';
import {Client, ClientEvents, restartClient} from './RealtimeEvents';

export default function BackendConnection(_props) {
	const [sockConnected, setSockConnected] = useState(false);

	useEffect(() => {
		ClientEvents.addEventListener('open', () => setSockConnected(true));
		ClientEvents.addEventListener('close', () => setSockConnected(false));
	});

	const reconnectBtn = (
		<button
			type='button' onClick={() => {
				restartClient();
			}}
		>Reconnect
		</button>
	);

	return (
		<div>
			<p style={{padding: 0, margin: 0}}>Backend <span style={{color: sockConnected ? 'green' : 'red'}}>{sockConnected ? 'Connected' : 'Disconnected'}</span></p>
			{!sockConnected && reconnectBtn}
		</div>
	);
}
