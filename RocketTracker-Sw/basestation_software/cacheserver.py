import argparse
import logging
from pathlib import Path
import asyncio
import time
import sys

from cacheproxy import server

logger = logging.getLogger(__name__)


def proxy_main(loop):

    backend_options = {
        k: v
        for k, v in dict(
            cache_name="mapcache",
            use_temp=False,
            expire_after=-1,
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

    runner = server.web.AppRunner(app)
    loop.run_until_complete(runner.setup())

    site = server.web.TCPSite(runner, 'localhost', 8085)
    loop.run_until_complete(site.start())


if __name__ == "__main__":
    proxy_main()
    while (1):
        time.sleep(100)
