import React from 'react';

export default function GridAdjuster({onAdjust, layout, size, target}) {
	const directions = ['x', 'y'];
	const wh = [layout.width, layout.height];

	let transform = {};
	if (Array.isArray(target)) {
		// Target[0] - axis of scaling
		// Target[1] - index of cell

		// Divisions to be adjusted
		const divs = layout.divisions[directions[target[0]]];
		const odivs = layout.divisions[directions[1 - target[0]]];
		transform = {
			left: target[0] === 1 ? (target[1] === 0 ? 0 : odivs[0]) : divs[target[1]],
			top: target[0] === 0 ? (target[1] === 1 ? 0 : odivs[0]) : divs[target[1]],
			width: (target[0]) === 1 ? (target[1] === 0 ? odivs[1] : layout.width - odivs[1]) : size,
			height: (target[0]) === 0 ? (target[1] === 0 ? odivs[1] : layout.width - odivs[1]) : size,
		};
	} else {
		transform = {
			left: target === 1 ? 0 : layout.divisions.x[0] - size,
			top: target === 0 ? 0 : layout.divisions.y[0] - size,
			width: target === 1 ? layout.width : size * 2,
			height: target === 0 ? layout.height : size * 2,
		};
	}

	return (
		<div
			draggable
			className='grid-adjuster'
			style={{
				position: 'absolute',
				/*
				Left: layout.flex === 0 ? 0 : layout.divisions.x[0] - size,
				top: layout.flex === 1 ? 0 : layout.divisions.y[0] - size,
				width: layout.flex === 0 ? layout.width : size * 2,
				height: layout.flex === 1 ? layout.height : size * 2,
				*/
				...transform,
			}}
			onDragEnd={e => onAdjust(e, target)}/>
	);
}
