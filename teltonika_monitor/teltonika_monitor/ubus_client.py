"""Minimal OpenWrt ubus JSON-RPC client using only stdlib.

Authenticates via session login and makes ubus calls over HTTPS.
Handles session expiry with automatic re-login.
"""

import json
import ssl
import time
import urllib.request
import urllib.error
from typing import Any


class UbusClientError(Exception):
    """Error communicating with ubus."""

    def __init__(self, message: str, code: int | None = None):
        super().__init__(message)
        self.code = code


class UbusClient:
    """Client for the OpenWrt ubus JSON-RPC API over HTTPS.

    Teltonika routers redirect HTTP to HTTPS, so this client uses
    HTTPS by default with certificate verification disabled (self-signed).
    """

    NULL_SESSION = '00000000000000000000000000000000'

    def __init__(
        self,
        host: str,
        username: str,
        password: str,
        port: int = 443,
        use_ssl: bool = True,
        verify_ssl: bool = False,
        timeout: float = 10.0,
    ):
        scheme = 'https' if use_ssl else 'http'
        self.url = f'{scheme}://{host}:{port}/ubus'
        self.username = username
        self.password = password
        self.timeout = timeout

        self.ssl_context = None
        if use_ssl:
            self.ssl_context = ssl.create_default_context()
            if not verify_ssl:
                self.ssl_context.check_hostname = False
                self.ssl_context.verify_mode = ssl.CERT_NONE

        self._session_id = None
        self._session_expires = 0
        self._rpc_id = 0

    def _next_id(self) -> int:
        self._rpc_id += 1
        return self._rpc_id

    def _post(self, payload: dict) -> dict:
        """Send a JSON-RPC POST request."""
        data = json.dumps(payload).encode()
        request = urllib.request.Request(
            self.url,
            data=data,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )

        try:
            with urllib.request.urlopen(
                request, timeout=self.timeout, context=self.ssl_context
            ) as response:
                return json.loads(response.read().decode())
        except urllib.error.HTTPError as e:
            body = e.read().decode() if e.fp else ''
            raise UbusClientError(
                f'HTTP {e.code} from {self.url}: {body}', code=e.code
            ) from e
        except urllib.error.URLError as e:
            raise UbusClientError(
                f'Connection error to {self.url}: {e.reason}'
            ) from e
        except (json.JSONDecodeError, OSError) as e:
            raise UbusClientError(
                f'Error reading response from {self.url}: {e}'
            ) from e

    def _login(self):
        """Authenticate and store session ID."""
        result = self._post({
            'jsonrpc': '2.0',
            'id': self._next_id(),
            'method': 'call',
            'params': [
                self.NULL_SESSION,
                'session', 'login',
                {'username': self.username, 'password': self.password},
            ],
        })

        if 'error' in result:
            raise UbusClientError(
                f'Login failed: {result["error"].get("message", "unknown")}',
                code=result['error'].get('code'),
            )

        ubus_result = result.get('result', [])
        if len(ubus_result) < 2 or ubus_result[0] != 0:
            raise UbusClientError(
                f'Login failed: unexpected result {ubus_result}'
            )

        session_data = ubus_result[1]
        self._session_id = session_data['ubus_rpc_session']
        expires = session_data.get('expires', 300)
        # Refresh 30s before expiry
        self._session_expires = time.monotonic() + expires - 30

    def _ensure_session(self):
        """Re-login if session is expired or missing."""
        if (
            self._session_id is None
            or time.monotonic() >= self._session_expires
        ):
            self._login()

    def call(self, obj: str, method: str, args: dict | None = None) -> Any:
        """Make a ubus call.

        Args:
            obj: ubus object (e.g., 'system', 'gsm.modem0')
            method: method name (e.g., 'board', 'get_signal_query')
            args: method arguments (default: {})

        Returns:
            The result data (second element of the result array).

        Raises:
            UbusClientError: On connection, auth, or ubus errors.
        """
        self._ensure_session()

        result = self._post({
            'jsonrpc': '2.0',
            'id': self._next_id(),
            'method': 'call',
            'params': [
                self._session_id,
                obj, method,
                args or {},
            ],
        })

        if 'error' in result:
            error = result['error']
            code = error.get('code')
            msg = error.get('message', 'unknown')
            # Access denied — session may have expired
            if code == -32002:
                self._session_id = None
                self._ensure_session()
                # Retry once
                result = self._post({
                    'jsonrpc': '2.0',
                    'id': self._next_id(),
                    'method': 'call',
                    'params': [
                        self._session_id,
                        obj, method,
                        args or {},
                    ],
                })
                if 'error' in result:
                    retry_error = result['error']
                    raise UbusClientError(
                        f'ubus {obj}.{method}: '
                        f'{retry_error.get("message", "unknown")}',
                        code=retry_error.get('code'),
                    )
            else:
                raise UbusClientError(
                    f'ubus {obj}.{method}: {msg}', code=code
                )

        ubus_result = result.get('result', [])
        if not ubus_result:
            raise UbusClientError(f'ubus {obj}.{method}: empty result')

        ret_code = ubus_result[0]
        if ret_code != 0:
            raise UbusClientError(
                f'ubus {obj}.{method}: error code {ret_code}',
                code=ret_code,
            )

        if len(ubus_result) > 1:
            return ubus_result[1]
        return {}

    def get_system_board(self) -> dict:
        """Get system board info (model, kernel, hostname)."""
        return self.call('system', 'board')

    def get_signal(self) -> dict:
        """Get cellular signal metrics (RSSI, RSRP, SINR, RSRQ)."""
        return self.call('gsm.modem0', 'get_signal_query')

    def get_modem_info(self) -> dict:
        """Get modem hardware info."""
        return self.call('gsm.modem0', 'info')

    def get_network_interfaces(self) -> dict:
        """Get all network interfaces."""
        return self.call('network.interface', 'dump')

    def get_mwan3_status(self) -> dict:
        """Get mwan3 failover status."""
        return self.call('mwan3', 'status')
