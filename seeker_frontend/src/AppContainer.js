import './App.css';
import React from 'react';
import Map from './Map';
import BackendConnection from './BackendConnection';
import Grid from './components/Grid';
import GridItem from './components/GridItem';
import Settings from './Settings';
import Terminal from './Terminal';
import MapRenderer from './components/MapRenderer';

function App() {
	return (
		<Grid>
			<GridItem index={1}>
				<Map/>
			</GridItem>
			<GridItem index={0}>
				<BackendConnection/>
			</GridItem>
			<GridItem index={2}>
				<Settings/>
			</GridItem>
			<GridItem index={3}>
				<Terminal/>
			</GridItem>
		</Grid>
	);
}

export default App;
