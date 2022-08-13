import React, {useState} from 'react';
import '../Tabs.css';

/**
 * Tab switching container
 * Switches between child components and displays clickable tab buttons using children's `name` attribute
 */

// TODO: Don't re-render tabs, just hide them

export default function Tabs({children}) {
	const [selected, setSelected] = useState(0);

	const titles = React.Children.map(children, child => child.props.name);

	return (
		<div className='tab-container'>
			<div className='tab-selectors'>
				{titles.map((t, i) => <button key={t} type='button' onClick={() => setSelected(i)}>{t}</button>)}
			</div>
			<hr style={{margin: 0, padding: 0}}/>
			<div className='selected-tab-container'>{children[selected]}</div>
		</div>
	);
}
