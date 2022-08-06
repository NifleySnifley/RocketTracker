import React, {useRef, useEffect, useState} from 'react';
import maplibregl from 'maplibre-gl';

export default function MapRenderer() {
	const mapContainer = useRef(null);
	const map = useRef(null);
	const [lng, setLng] = useState(139.753);
	const [lat, setLat] = useState(35.6844);
	const [zoom, setZoom] = useState(14);
	const API_KEY = process.env.REACT_APP_MAPTILER_API_KEY;

	// TODO: Global state for sensor data and events
	useEffect(() => {
		if (map.current) {
			return;
		}

		map.current = new maplibregl.Map({
			container: mapContainer.current,
			style: `https://api.maptiler.com/maps/streets/style.json?key=${API_KEY}`,
			center: [lng, lat],
			zoom,
		});

		map.current.addControl(new maplibregl.NavigationControl(), 'top-right');

		new maplibregl.Marker({color: '#FF0000'})
			.setLngLat([139.7525, 35.6846])
			.addTo(map.current);
	});

	return (
		<div className='map-wrap'>
			<div ref={mapContainer} className='map'/>
		</div>
	);
}
