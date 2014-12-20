/*
 * Socket.cpp
 *
 *  Created on: 14.11.2008
 *      Author:
 */

// Implementation of the Socket class.

#include "mocap_optitrack/socket.h"
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <iostream>
#include <stdio.h>
#include <sstream>

#include <ros/ros.h>

UdpMulticastSocket::UdpMulticastSocket( const int local_port, const std::string multicast_ip ) 
{
  // Create a UDP socket
  ROS_INFO( "Creating socket..." );
  m_socket = socket( AF_INET, SOCK_DGRAM, 0 );
  if( m_socket < 0 )
    throw SocketException( strerror( errno ) );

  // Allow reuse of local addresses
  ROS_INFO( "Setting socket options..." );
  int option_value = 1;
  int result = setsockopt( m_socket, SOL_SOCKET, SO_REUSEADDR, (void*)&option_value, sizeof( int ) );
  if( result == -1 )
  {
    std::stringstream error;
    error << "Failed to set socket option: ";
    switch( errno )
    {
      case EBADF:
  error << "EBADF";
  break;
      case EFAULT:
  error << "EFAULT";
  break;
      case EINVAL:
  error << "EINVAL";
  break;
      case ENOPROTOOPT:
  error << "ENOPROTOOPT";
  break;
      case ENOTSOCK:
  error << "ENOTSOCK";
  break;
      default:
  error << "unknown error";
  break;
    }
    throw SocketException( error.str().c_str() );    
  }
  
  // Fill struct for local address
  memset ( &m_local_addr, 0, sizeof ( m_local_addr ) );
  m_local_addr.sin_family = AF_INET;
  m_local_addr.sin_addr.s_addr = htonl( INADDR_ANY );
  m_local_addr.sin_port = htons( local_port );
  ROS_INFO( "Local address: %s:%i", inet_ntoa( m_local_addr.sin_addr ), ntohs( m_local_addr.sin_port ) );

  // Bind the socket
  ROS_INFO( "Binding socket to local address..." );
  result = bind( m_socket, (sockaddr*)&m_local_addr, sizeof( m_local_addr ) );
  if( result == -1 )
  {
    std::stringstream error;
    error << "Failed to bind socket to local address:" << strerror( errno );
    throw SocketException( error.str().c_str() );
  }
  
  // Join multicast group
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr( multicast_ip.c_str() );
  mreq.imr_interface = m_local_addr.sin_addr;
  ROS_INFO( "Joining multicast group %s...", inet_ntoa( mreq.imr_multiaddr ) );

  result = setsockopt(m_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq));
  if( result == -1 )
  {
    std::stringstream error;
    error << "Failed to set socket option: ";
    switch( errno )
    {
      case EBADF:
  error << "EBADF";
  break;
      case EFAULT:
  error << "EFAULT";
  break;
      case EINVAL:
  error << "EINVAL";
  break;
      case ENOPROTOOPT:
  error << "ENOPROTOOPT";
  break;
      case ENOTSOCK:
  error << "ENOTSOCK";
  break;
      default:
  error << "unknown error";
  break;
    }
    throw SocketException( error.str().c_str() );    
  }
    
  // Make socket non-blocking
  ROS_INFO( "Enabling non-blocking I/O" );
  int flags = fcntl( m_socket, F_GETFL , 0 );
  result = fcntl(m_socket, F_SETFL, flags | O_NONBLOCK);
  if( result == -1 )
  {
    std::stringstream error;
    error << "Failed to enable non-blocking I/O: " << strerror( errno );
    throw SocketException( error.str().c_str() );
  }
}

UdpMulticastSocket::~UdpMulticastSocket()
{
  close( m_socket );
}

int UdpMulticastSocket::recv()
{
  memset ( buf, 0, MAXRECV + 1 );

  sockaddr_in remote_addr;
  int addr_len = sizeof(struct sockaddr);
  int status = recvfrom(
    m_socket,
    buf,
    MAXRECV,
    0,
    (sockaddr *)&remote_addr,
    (socklen_t*)&addr_len);

  if( status > 0 )
    ROS_INFO( "%4i bytes received from %s:%i", status, inet_ntoa( remote_addr.sin_addr ), ntohs( remote_addr.sin_port ) );
  else if( status == 0 )
    ROS_INFO( "Connection closed by peer" );

  return status;
}
