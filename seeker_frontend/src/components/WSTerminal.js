import React, {useEffect, useState} from 'react';
import {RealtimeEvents, ClientEvents, Client} from '../RealtimeEvents';

export default function WSTerminal(_props) {
	const [data, setData] = useState('');
	useEffect(() => {
		ClientEvents.addEventListener('message', m => setData(data + '\n' + m.detail.message.data));
	});

	return (
		<code style={{whiteSpace: 'pre-wrap', overflow: 'scroll', width: '100%', height: '100%'}}>{data}</code>
	);
}
