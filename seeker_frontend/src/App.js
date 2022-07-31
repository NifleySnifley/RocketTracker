import './App.css';
import {FontAwesomeIcon} from '@fortawesome/react-fontawesome';
import * as Icons from '@fortawesome/free-solid-svg-icons';
import React from 'react';

function App() {
	return (
		<div className='App'>
			<p>Hello!</p><br/>
			Ready for some <FontAwesomeIcon icon={Icons.faRocket}/>?
		</div>
	);
}

export default App;
