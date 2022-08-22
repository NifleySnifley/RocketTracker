import React, {useEffect, useState} from 'react';
import {Events} from '../RealtimeEvents';

export default function WSTerminal(_props) {
	const [data, setData] = useState('');

	useEffect(() => {
		Events.addEventListener('message', m => {
			setData(data + '\n' + m.detail.message);
		});
	});

	return (
		<code style={{whiteSpace: 'pre-wrap', overflow: 'scroll', width: '100%', height: '100%'}}>{data}</code>
	);
}
