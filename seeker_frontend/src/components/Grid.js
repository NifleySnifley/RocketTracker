import React, {createContext, useRef, useState} from 'react';

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
	flex: 'height',
});

export default function Grid(props) {
	const [layout, setLayout] = useState(defaultGridState());
	const resizerAdded = useRef(false);

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
			setLayout(defaultGridState());
		}

		if (!resizerAdded.current) {
			window.addEventListener('resize', handleResize);
			resizerAdded.current = true;
		}
	});

	return (
		<div>
			{fixedChildren}
		</div>
	);
}
