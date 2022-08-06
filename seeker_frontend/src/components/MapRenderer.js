import React, {useRef, useEffect, useState} from 'react';
import maplibregl from 'maplibre-gl';

export default function MapRenderer() {
	const mapContainer = useRef(null);
	const map = useRef(null);
	const [lng, setLng] = useState(-93.258133);
	const [lat, setLat] = useState(44.986656);
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

		map.current.on('load', () => {
			map.current.addSource('wms-gisdata-mn', {
				type: 'raster',
				// https://maplibre.org/maplibre-gl-js-docs/style-spec/sources/
				tiles: [
					// 'https://imageserver.gisdata.mn.gov/cgi-bin/mncomp?bbox={bbox-epsg-3857}&service=WMS&version=1.1.1&request=GetMap&srs=EPSG:3857&transparent=true&width=256&height=256&layers=mncomp',
					'https://imageserver.gisdata.mn.gov/cgi-bin/wms?bbox={bbox-epsg-3857}&service=WMS&version=1.1.1&request=GetMap&srs=EPSG:3857&transparent=true&width=256&height=256&layers=fsa2021',
				],
				tileSize: 256,
			});
			map.current.addLayer(
				{
					id: 'gisdata-mn',
					type: 'raster',
					source: 'wms-gisdata-mn',
					paint: {},
				},
				'aeroway_fill',
			);
		});
	});

	return (
		<div className='map-wrap'>
			<div ref={mapContainer} className='map'/>
		</div>
	);
}
