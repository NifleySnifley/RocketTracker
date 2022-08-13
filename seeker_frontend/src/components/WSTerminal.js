import React, {useEffect, useState} from 'react';
import {ClientEvents, Events} from '../RealtimeEvents';

export default function WSTerminal(_props) {
	const [data, setData] = useState('');

	useEffect(() => {
		// ClientEvents.addEventListener('message', m => {
		// 	setData(data + '\n' + JSON.stringify(m.detail, null, 4));
		// });

		Events.addEventListener('Debug', e => {
			console.log('Got one!!');
		});
	});

	return (
		<code style={{whiteSpace: 'pre-wrap', overflow: 'scroll', width: '100%', height: '100%'}}>{data}</code>
	);
}
