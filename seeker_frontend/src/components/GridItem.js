import React, {useContext} from 'react';
import GridState from './Grid';

/**
 * Item in a grid
 * MUST be child of a {@link GridItem}
 * @param {{index: Number}} props
 * @returns Grid element
 */
export default function GridItem({children, layout, index}) {
	let x = 0;
	let y = 0;
	let width = 0;
	let height = 0;

	const xi = index & 1;
	const yi = (index & 2) >> 1;

	const dx = layout.divisions.x[yi];
	const dy = layout.divisions.y[xi];

	if (xi === 0) {
		x = 0;
		width = dx;
	} else {
		x = dx;
		width = layout.width - dx;
	}

	if (yi === 0) {
		y = 0;
		height = dy;
	} else {
		y = dy;
		height = layout.height - dy;
	}

	return (
		<div
			className='grid-item' style={{
				position: 'absolute',
				left: x,
				top: y,
				width, height,
			}}
		>
			{children}
		</div>
	);
}
