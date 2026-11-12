"""Explain network failures without confusing them with UART or firmware errors."""
import errno
import functools
import socket


class PiConnectionError(ConnectionError):
    pass


def connection_failure(detail):
    return PiConnectionError(f"Pi connection lost or timed out ({detail}). Check or reconnect Ethernet, then retry.")


def guard_pi_connection(function):
    @functools.wraps(function)
    def wrapped(*args, **kwargs):
        try:
            return function(*args, **kwargs)
        except PiConnectionError:
            raise
        except (TimeoutError, ConnectionError, socket.gaierror, EOFError) as exc:
            raise connection_failure(str(exc) or type(exc).__name__) from exc
        except OSError as exc:
            if (isinstance(getattr(exc, "errors", None), dict) or
                exc.errno in {errno.ENETDOWN, errno.ENETUNREACH, errno.EHOSTUNREACH,
                             errno.ECONNABORTED, errno.ECONNRESET, errno.ECONNREFUSED,
                             errno.EPIPE, errno.ETIMEDOUT}):
                raise connection_failure(str(exc)) from exc
            raise
    return wrapped
