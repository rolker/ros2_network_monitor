"""Minimal RouterOS 7 REST API client using only stdlib."""

import json
import ssl
import urllib.request
import urllib.error
import base64
from typing import Any


class RouterOSClientError(Exception):
    """Error communicating with RouterOS device."""


class RouterOSClient:
    """Client for the RouterOS 7 REST API.

    Uses HTTP basic auth. Supports both HTTP and HTTPS (use_ssl parameter).
    SSL certificate verification is disabled by default since MikroTik
    devices typically use self-signed certs.
    """

    def __init__(
        self,
        host: str,
        username: str,
        password: str,
        port: int = 80,
        use_ssl: bool = False,
        verify_ssl: bool = False,
        timeout: float = 10.0,
    ):
        scheme = 'https' if use_ssl else 'http'
        self.base_url = f'{scheme}://{host}:{port}/rest'
        self.timeout = timeout

        # Build auth header
        credentials = base64.b64encode(
            f'{username}:{password}'.encode()
        ).decode()
        self.headers = {
            'Authorization': f'Basic {credentials}',
            'Content-Type': 'application/json',
        }

        # SSL context for self-signed certs (only used when use_ssl=True)
        self.ssl_context = None
        if use_ssl:
            self.ssl_context = ssl.create_default_context()
            if not verify_ssl:
                self.ssl_context.check_hostname = False
                self.ssl_context.verify_mode = ssl.CERT_NONE

    def get(self, path: str) -> Any:
        """GET a REST API endpoint. Returns parsed JSON response.

        Args:
            path: API path (e.g., '/interface' for GET /rest/interface)

        Returns:
            Parsed JSON response (usually a list of dicts).

        Raises:
            RouterOSClientError: On connection or API errors.
        """
        url = f'{self.base_url}{path}'
        request = urllib.request.Request(url, headers=self.headers)

        try:
            with urllib.request.urlopen(
                request, timeout=self.timeout, context=self.ssl_context
            ) as response:
                return json.loads(response.read().decode())
        except urllib.error.HTTPError as e:
            body = e.read().decode() if e.fp else ''
            raise RouterOSClientError(
                f'HTTP {e.code} from {url}: {body}'
            ) from e
        except urllib.error.URLError as e:
            raise RouterOSClientError(
                f'Connection error to {url}: {e.reason}'
            ) from e
        except (json.JSONDecodeError, OSError) as e:
            raise RouterOSClientError(
                f'Error reading response from {url}: {e}'
            ) from e

    def get_interfaces(self) -> list[dict]:
        """Get all network interfaces with traffic counters."""
        return self.get('/interface')

    def get_wireless_registrations(self) -> list[dict]:
        """Get wireless registration table (connected wireless clients/APs)."""
        return self.get('/interface/wireless/registration-table')

    def get_system_identity(self) -> dict:
        """Get system identity (device name)."""
        return self.get('/system/identity')

    def get_system_resource(self) -> dict:
        """Get system resource usage (CPU, memory, uptime)."""
        return self.get('/system/resource')
