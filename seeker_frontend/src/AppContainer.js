import './App.css';
import React from 'react';
import BackendConnection from './BackendConnection';
import Grid from './components/Grid';
import GridItem from './components/GridItem';
import Settings from './Settings';
import Terminal from './Terminal';
import Tabs from './components/Tabs';
import Viewport from './Viewport';

/**
 * Container for holding all components in the app.
 * This is the top of the hierarchy
 * @returns The app!
 */
function App() {
	return (
		<Grid>
			<GridItem index={1}>
				<Tabs>
					<Viewport name='Map 2D'/>
					<div name='Flight Data'>WIP</div>
				</Tabs>
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
