/* eslint-disable react/jsx-no-bind */
import * as Icons from '@fortawesome/free-solid-svg-icons';
import {FontAwesomeIcon} from '@fortawesome/react-fontawesome';
import React, {createContext, useRef, useState} from 'react';
import GridAdjuster from './GridAdjuster';

/**
 * @typedef {width: number, height:number, divisions:{x:[number], y:[number]}} GridLayout
 */

/**
 * Generates a new layout
 * @param {number} flex axis to allow flexible adjustment in (0:x, 1:y)
 * @returns {GridLayout} layout
 */
const defaultGridState = (flex = 0) => ({
	width: document.documentElement.clientWidth,
	height: document.documentElement.clientHeight,
	divisions: {
		x: flex === 1 ? [
			document.documentElement.clientWidth / 3,
			document.documentElement.clientWidth / 3,
		] : [
			document.documentElement.clientWidth / 3,
			document.documentElement.clientWidth / 2,
		],
		y: flex === 1 ? [
			document.documentElement.clientHeight / 3,
			document.documentElement.clientHeight / 3 * 2.5,
		] : [
			document.documentElement.clientHeight / 3 * 2,
			document.documentElement.clientHeight / 3 * 2,
		],
	},
	flex,
});

/**
 * Scales a layout with an out-of-date size to match the dimensions of the window currently.
 * @param {GridLayout} layout layout to scale
 * @returns {GridLayout} Properly sized layout proportional to the one provided
 */
function scaleLayout(layout) {
	const newWH = {
		width: document.documentElement.clientWidth,
		height: document.documentElement.clientHeight,
	};

	const newDivs = {
		x: layout.divisions.x.map(s => (s / layout.width) * newWH.width),
		y: layout.divisions.y.map(s => (s / layout.height) * newWH.height),
	};

	return {divisions: newDivs, ...newWH, flex: layout.flex};
}

/**
 * Tries to load a layout from localStorage and falls back to creating a new default layout.
 * @returns {GridLayout} layout
 */
function loadLayout() {
	if (window.localStorage.getItem('layout')) {
		return scaleLayout(JSON.parse(window.localStorage.getItem('layout')));
	}

	return defaultGridState();
}

/**
 * Adjustable grid layout container
 * Holds (4) {@link GridItem} items
 * @param {{children: [React.Component], }}} props
 */
export default function Grid(props) {
	// eslint-disable-next-line react/hook-use-state
	const [layout, setLayoutRaw] = useState(loadLayout());
	const resizerAdded = useRef(false);

	const setLayout = l => {
		window.localStorage.setItem('layout', JSON.stringify(l));
		setLayoutRaw(l);
	};

	/** @type {[React.Component]} */
	const fixedChildren = React.Children.map(props.children, child => {
		// Checking isValidElement is the safe way and avoids a typescript
		// error too.
		if (React.isValidElement(child)) {
			return React.cloneElement(child, {layout, ...child.props});
		}

		return child;
	});

	React.useEffect(() => {
		function handleResize() {
			setLayout(scaleLayout(loadLayout()));
		}

		if (!resizerAdded.current) {
			window.addEventListener('resize', handleResize);
		}

		return () => window.removeEventListener('resize', handleResize);
	});

	/**
	 * Called when a divider is dragged
	 * @param {DragEvent} e
	 * @param {number} dir
	 * TODO: if dir is an array (path) use it for flex dividers
	 */
	function drag(e, dir) {
		const newLayout = {...layout};
		const dirs = ['x', 'y'];

		if (Array.isArray(dir)) {
			newLayout.divisions[dirs[dir[0]]][dir[1]] = e[`client${dirs[dir[0]].toUpperCase()}`];
		} else {
			newLayout.divisions[dirs[dir]] = [0, 0].map(v => e[`client${dirs[dir].toUpperCase()}`]);
		}

		setLayout(newLayout);
	}

	const adjusterSize = 4;

	return (
		<div>
			{fixedChildren}
			<GridAdjuster layout={layout} size={adjusterSize} target={1 - layout.flex} onAdjust={drag}/>
			<GridAdjuster layout={layout} size={adjusterSize} target={[layout.flex, 0]} onAdjust={drag}/>
			<GridAdjuster layout={layout} size={adjusterSize} target={[layout.flex, 1]} onAdjust={drag}/>
			<FontAwesomeIcon
				icon={layout.flex === 1 ? Icons.faArrowsLeftRight : Icons.faArrowsUpDown} color='white'
				style={{
					backgroundColor: 'black',
					borderRadius: '0.6em',
					border: '0.1em solid black',
					width: '1em',
					height: '1em',
					position: 'absolute',
					top: `calc(${layout.flex === 0 ? layout.divisions.y[0] : layout.height / 2}px - 0.6em)`,
					left: `calc(${layout.flex === 1 ? layout.divisions.x[0] : layout.width / 2}px - 0.6em)`,
				}}
				fontSize='1em'
				onClick={e => setLayout(defaultGridState(1 - layout.flex))}
			/>
		</div>
	);
}
