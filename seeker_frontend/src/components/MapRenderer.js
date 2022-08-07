import React, {useRef, useEffect, useState} from 'react';
import maplibregl, {Map} from 'maplibre-gl';

/**
 * Map renderer component
 * @param {Object} props Props
 * @param {[maplibregl.CustomLayerInterface]} props.customLayers Custom layers to overlay on the map
 * @returns MapLibreGL map renderer!
 */
export default function MapRenderer({customLayers}) {
	const mapContainer = useRef(null);
	/** @type {React.MutableRefObject<Map>} */
	const map = useRef(null);
	/** @type {React.MutableRefObject<ResizeObserver>} */
	const resizeObserver = useRef(null);

	const [lng, setLng] = useState(-93.258133);
	const [lat, setLat] = useState(44.986656);
	const [zoom, setZoom] = useState(14);
	const API_KEY = process.env.REACT_APP_MAPTILER_API_KEY;

	useEffect(() => {
		if (!map.current) {
			map.current = new maplibregl.Map({
				container: mapContainer.current,
				style: `https://api.maptiler.com/maps/streets/style.json?key=${API_KEY}`,
				center: [lng, lat],
				zoom,
				antialias: true,
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
						'background-opacity': 0,
						paint: {},
					},
					'aeroway_fill',
				);

				for (const customLayer of (customLayers || [])) {
					map.current.addLayer(customLayer, 'building');
				}
			});
		}

		if (!resizeObserver.current) {
			resizeObserver.current = new ResizeObserver(items => {
				map.current.resize();
			});

			resizeObserver.current.observe(mapContainer.current);
		}
	});

	return (
		<div className='map-wrap'>
			<div ref={mapContainer} className='map'/>
		</div>
	);
}
