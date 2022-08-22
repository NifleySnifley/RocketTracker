import React, {useEffect, useState} from 'react';
import {Client, Events, restartClient} from './RealtimeEvents';
import TrackerList from './TrackerList';

export default function BackendConnection(_props) {
	const [sockConnected, setSockConnected] = useState(false);

	useEffect(() => {
		setSockConnected(Client.readyState === Client.OPEN);
		Events.addEventListener('open', () => setSockConnected(true));
		Events.addEventListener('close', () => setSockConnected(false));
	}, []);

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
			<hr/>
			<TrackerList/>
		</div>
	);
}
