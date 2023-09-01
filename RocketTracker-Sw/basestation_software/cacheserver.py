import argparse
import logging
from pathlib import Path
import asyncio

from cacheproxy import server

logger = logging.getLogger(__name__)

PROXY_QUIT = False
def proxy_main():
    # args = parser.parse_args()

    # if args.quiet:
    #     pass
    # elif args.debug:
    #     logging.basicConfig(level=logging.DEBUG, format="%(message)s\n")
    # else:
    #     logging.basicConfig(level=logging.INFO, format="%(message)s\n")
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    backend_options = {
        k: v
        for k, v in dict(
            cache_name="mapcache",
            use_temp=False,
            expire_after=-1,
            # urls_expire_after={
            #     "*.fillmurray.com": -1
            # },
            allowed_codes=[200],
            allowed_methods=["GET", "POST", "HEAD"],
            include_headers=True,
            ignored_params=[],
            timeout=10,
        ).items()
        if v is not None
    }


    # NOTE: https://aiohttp-client-cache.readthedocs.io/en/latest/modules/aiohttp_client_cache.backends.sqlite.html#aiohttp_client_cache.backends.sqlite.SQLiteBackend
    from aiohttp_client_cache import SQLiteBackend

    cache = SQLiteBackend(**backend_options)
    logger.info(f"Cache database: {cache.responses.filename}")


    app = server.web.Application()
    handler = server.HandlerWithCache(cache)
    app.add_routes([server.web.get("/{url:.+}", handler.handle)])

    # server.web.run_app(app, host="0.0.0.0", port=8080)
    
    runner = server.web.AppRunner(app)
    loop.run_until_complete(runner.setup())
    site = server.web.TCPSite(runner, 'localhost', 8080)
    loop.run_until_complete(site.start())

    # while not PROXY_QUIT:
    #     loop.run_until_complete(asyncio.sleep(1))  # sleep forever