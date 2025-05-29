#!/usr/bin/env python

import ssl
import time
import base64
import socket
import select
import logging

from .rtcm_parser import RTCMParser

_CHUNK_SIZE = 1024
_SOURCETABLE_RESPONSES = [
  'SOURCETABLE 200 OK'
]
_SUCCESS_RESPONSES = [
  'ICY 200 OK',
  'HTTP/1.0 200 OK',
  'HTTP/1.1 200 OK'
]
_UNAUTHORIZED_RESPONSES = [
  '401'
]

class NTRIPClient:

  # Public constants
  RECONNECT_FIRST_FREQUENCY_INTERVAL = 5*60
  DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS = 5
  RECONNECT_ATEMPT_WAIT_SECONDS = 5
  DEFAULT_RECONNECT_ATTEMPT_MAX = \
    RECONNECT_FIRST_FREQUENCY_INTERVAL/DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS
  DEFAULT_RTCM_TIMEOUT_SECONDS = 4

  def __init__(self, 
               host, 
               port, 
               mountpoint, 
               ntrip_version, 
               username, 
               password, 
               logerr=logging.error, 
               logwarn=logging.warning, 
               loginfo=logging.info, 
               logdebug=logging.debug):
    self._logerr = logerr
    self._logwarn = logwarn
    self._loginfo = loginfo
    self._logdebug = logdebug

    self._host = host
    self._port = port
    self._mountpoint = mountpoint
    self._ntrip_version = ntrip_version
    if username is not None and password is not None:
      self._basic_credentials = base64.b64encode(f'{username}:{password}'\
                                                 .encode()).decode('utf-8')
    else:
      self._basic_credentials = None

    self._raw_socket = None
    self._server_socket = None

    self.rtcm_parser = RTCMParser(
      logerr=logerr,
      logwarn=logwarn,
      loginfo=loginfo,
      logdebug=logdebug
    )

    self.ssl = False
    self.cert = None
    self.key = None
    self.ca_cert = None

    self._shutdown = False
    self._connected = False

    # Private reconnect info
    self._reconnect_attempt_count = 0
    self._nmea_send_failed_count = 0
    self._nmea_send_failed_max = 5
    self._read_zero_bytes_count = 0
    self._read_zero_bytes_max = 5
    self._first_rtcm_received = False
    self._recv_rtcm_last_packet_timestamp = 0

    # Public reconnect info
    self.reconnect_attempt_max = self.DEFAULT_RECONNECT_ATTEMPT_MAX
    self.reconnect_attempt_wait_seconds = self.DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS
    self.reconnect_attempt_wait_interval = self.RECONNECT_ATEMPT_WAIT_SECONDS
    self.rtcm_timeout_seconds = self.DEFAULT_RTCM_TIMEOUT_SECONDS
    self.reconnect_count = 0

  def connect(self):
    self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self._server_socket.settimeout(5)

    # Connect the socket to the server
    try:
      self._server_socket.connect((self._host, self._port))
    except Exception:
      self._logerr(
        f'Unable to connect socket to server at http://{self._host}:{self._port}.'+\
            ' Check connectivity')
      return False

    if self.ssl:
      # Configre the context based on the config
      self._ssl_context = ssl.create_default_context()
      if self.cert:
        self._ssl_context.load_cert_chain(self.cert, self.key)
      if self.ca_cert:
        self._ssl_context.load_verify_locations(self.ca_cert)

      # Save the old socket for later just in case, and create a new SSL socket
      self._raw_socket = self._server_socket
      self._server_socket = self._ssl_context.wrap_socket(
        self._raw_socket, server_hostname=self._host)

    # Send the HTTP Request
    try:
      self._server_socket.send(self._form_request())
    except Exception as e:
      self._logerr(
        f'Unable to send request to server at http://{self._host}:{self._port}')
      self._logerr(f'Exception: {e}')
      return False

    # Get the response from the server
    response = ''
    try:
      response = self._server_socket.recv(_CHUNK_SIZE).decode('ISO-8859-1')
    except Exception as e:
      self._logerr(
        f'Unable to read response from server at http://{self._host}:{self._port}')
      self._logerr(f'Exception: {e}')
      return False

    # Properly handle the response
    if any(success in response for success in _SUCCESS_RESPONSES):
      self._connected = True

    # Some debugging hints about the kind of error we received
    known_error = False
    if any(sourcetable in response for sourcetable in _SOURCETABLE_RESPONSES):
      self._logwarn('Received sourcetable response from the server. '+\
                    'This probably means the mountpoint specified is not valid')
      known_error = True
    elif any(unauthorized in response for unauthorized in _UNAUTHORIZED_RESPONSES):
      self._logwarn('Received unauthorized response from the server. '+\
                    'Check your username, password, and mountpoint to make sure they are correct.')
      known_error = True
    elif not self._connected and (self._ntrip_version is None or self._ntrip_version == ''):
      self._logwarn('Received unknown error from the server. Note that the NTRIP version '+\
                    'was not specified in the launch file. '+\
                    'This is not necesarilly the cause of this error, '+\
                    'but it may be worth checking your NTRIP casters documentation '+\
                    'to see if the NTRIP version needs to be specified.')
      known_error = True

    # Wish we could just return from the above checks, 
    # but some casters return both a success and an error in the response
    # If we received any known error, 
    # even if we received a success it should be considered a failure
    if known_error or not self._connected:
      self._logerr('Invalid response received from '+\
                   f'http://{self._host}:{self._port}/{self._mountpoint}. Check if base is UP')
      return False
    else:
      if not self.reconnect_count >= 1:
        self._loginfo(f'Connected to http://{self._host}:{self._port}/{self._mountpoint}')
      return True


  def disconnect(self):
    # Disconnect the socket
    self._connected = False
    try:
      if self._server_socket:
        self._server_socket.shutdown(socket.SHUT_RDWR)
      if self._raw_socket:
        self._raw_socket.shutdown(socket.SHUT_RDWR)
    except Exception as e:
      self._logdebug('Encountered exception when shutting'+\
                     ' down the socket. This can likely be ignored')
      self._logdebug(f'Exception: {e}')
    try:
      if self._server_socket:
        self._server_socket.close()
      if self._raw_socket:
        self._raw_socket.close()
    except Exception as e:
      self._logdebug('Encountered exception when closing the socket. This can likely be ignored')
      self._logdebug(f'Exception: {e}')

  def reconnect(self):
    self.reconnect_count += 1
    if self._connected:
      while not self._shutdown:
        self._reconnect_attempt_count += 1
        self.disconnect()
        connect_success = self.connect()
        if not connect_success and self._reconnect_attempt_count < self.reconnect_attempt_max:
          self._logerr(f'Reconnect to http://{self._host}:{self._port} failed. '+\
                       f'Retrying in { self.reconnect_attempt_wait_seconds} seconds')
          time.sleep(self.reconnect_attempt_wait_seconds)
        elif not connect_success and self._reconnect_attempt_count >= self.reconnect_attempt_max:
          self._logerr(f'Reconnect to http://{self._host}:{self._port} failed. '+\
                       f'Retrying in { self.reconnect_attempt_wait_interval} seconds')
          time.sleep(self.reconnect_attempt_wait_interval)   
        elif connect_success:
          self._reconnect_attempt_count = 0
          break
    else:
      self._logdebug('Reconnect called while still connected, ignoring')


  def recv_rtcm(self):
    if not self._connected:
      self._logwarn(
        'RTCM requested before client was connected, returning empty list')
      return []

    # If it has been too long since we received an RTCM packet, reconnect
    if time.time() - self.rtcm_timeout_seconds >= \
        self._recv_rtcm_last_packet_timestamp and self._first_rtcm_received:
      self._logwarn(f'RTCM data not received for {self.rtcm_timeout_seconds} '+\
                    'seconds, reconnecting')
      self.reconnect()
      self._first_rtcm_received = False

    # Check if there is any data available on the socket
    read_sockets, _, _ = select.select([self._server_socket], [], [], 0)
    if not read_sockets:
      return []

    # Since we only ever pass the server socket to the list of 
    # read sockets, we can just read from that
    # Read all available data into a buffer
    data = b''
    while True:
      try:
        chunk = self._server_socket.recv(_CHUNK_SIZE)
        data += chunk
        if len(chunk) < _CHUNK_SIZE:
          break
      except Exception:
        self._logerr(f'Error while reading {_CHUNK_SIZE} bytes from socket')
        if not self._socket_is_open():
          self._logerr('Socket appears to be closed. Reconnecting')
          self.reconnect()
          return []
        break
    self._logdebug(f'Read {len(data)} bytes')

    # If 0 bytes were read from the socket even though we 
    # were told data is available multiple times,
    # it can be safely assumed that we can reconnect as the server has closed the connection
    if len(data) == 0:
      self._read_zero_bytes_count += 1
      if self._read_zero_bytes_count >= self._read_zero_bytes_max:
        self._logwarn(
          'Reconnecting because we received 0 bytes from the socket even'+\
            f' though it said there was data available {self._read_zero_bytes_count} times')
        self.reconnect()
        self._read_zero_bytes_count = 0
        return []
    else:
      self._recv_rtcm_last_packet_timestamp = time.time()
      self._first_rtcm_received = True

    return self.rtcm_parser.parse(data) if data else []


  def shutdown(self):
    self._shutdown = True
    self.disconnect()


  def _form_request(self):
    if self._ntrip_version is not None and self._ntrip_version != '':
      request_str = f'GET /{self._mountpoint} HTTP/1.0\r\nNtrip-Version: '+\
        f'{self._ntrip_version}\r\nUser-Agent: NTRIP ntrip_client_ros\r\n'
    else:
      request_str = f'GET /{self._mountpoint} HTTP/1.0\r\nUser-Agent: NTRIP ntrip_client_ros\r\n'
    if self._basic_credentials is not None:
      request_str += f'Authorization: Basic {self._basic_credentials}\r\n'
    request_str += '\r\n'
    return request_str.encode('utf-8')


  def _socket_is_open(self):
    try:
      # this will try to read bytes without blocking 
      # and also without removing them from buffer (peek only)
      data = self._server_socket.recv(_CHUNK_SIZE, socket.MSG_DONTWAIT | socket.MSG_PEEK)
      if len(data) == 0:
        return False
    except BlockingIOError:
      return True  # socket is open and reading from it would block
    except ConnectionResetError:
      self._logwarn('Connection reset by peer')
      return False  # socket was closed for some other reason
    except socket.timeout:
      return True  # timeout likely means that the socket is still open
    except Exception as e:
      self._logwarn('Socket appears to be closed')
      self._logwarn(f'Exception: {e}')
      return False
    return True