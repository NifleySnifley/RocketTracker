/* eslint-disable react/jsx-no-bind */
import React, {createContext, useRef, useState} from 'react';
import GridAdjuster from './GridAdjuster';

const defaultGridState = () => ({
	width: document.documentElement.clientWidth,
	height: document.documentElement.clientHeight,
	divisions: {
		x: [
			document.documentElement.clientWidth / 3,
			document.documentElement.clientWidth / 3,
		],
		y: [
			document.documentElement.clientHeight / 2,
			document.documentElement.clientHeight / 3 * 2,
		],
	},
	flex: 1,
});

function scaleLayout(layout) {
	const newWH = {
		width: document.documentElement.clientWidth,
		height: document.documentElement.clientHeight,
	};

	const newDivs = {
		x: layout.divisions.x.map(s => (s / layout.width) * newWH.width),
		y: layout.divisions.y.map(s => (s / layout.height) * newWH.height),
	};

	console.log({...newDivs, ...newWH, flex: layout.flex});
	return {divisions: newDivs, ...newWH, flex: layout.flex};
}

function loadLayout() {
	if (window.localStorage.getItem('layout')) {
		return JSON.parse(window.localStorage.getItem('layout'));
	}

	return defaultGridState();
}

// Add reactive bars to adjust dividers and flex direction
export default function Grid(props) {
	// eslint-disable-next-line react/hook-use-state
	const [layout, setLayoutRaw] = useState(loadLayout());
	const resizerAdded = useRef(false);
	const divBar = useRef(null);

	const setLayout = l => {
		window.localStorage.setItem('layout', JSON.stringify(l));
		setLayoutRaw(l);
	};

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
			resizerAdded.current = true;
		}
	});

	/**
	 * Called when a divider is dragged
	 * @param {DragEvent} e
	 * @param {number} dir
	 * TODO: if dir is an array (path) use it for flex dividers
	 */
	function drag(e, dir) {
		const newLayout = {...layout};
		const dirs = ['y', 'x'];

		if (Array.isArray(dir)) {
			newLayout.divisions[dirs[1 - dir[0]]][dir[1]] = e[`client${dirs[1 - dir[0]].toUpperCase()}`];
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
			{/* <div
				ref={divBar}
				draggable
				className='grid-adjuster'
				style={{
					position: 'absolute',
					left: layout.flex === 0 ? 0 : layout.divisions.x[0] - adjusterSize,
					top: layout.flex === 1 ? 0 : layout.divisions.y[0] - adjusterSize,
					width: layout.flex === 0 ? layout.width : adjusterSize * 2,
					height: layout.flex === 1 ? layout.height : adjusterSize * 2,
				}}
				onDragEnd={e => drag(e, layout.flex)}/> */}
		</div>
	);
}
