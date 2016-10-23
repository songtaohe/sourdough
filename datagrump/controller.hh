#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <cstdint>
#include <pthread.h>
/* Congestion controller interface */

class Controller
{
private:
  bool debug_; /* Enables debugging output */
public:
  /* Add member variables here */
  float window_size_float;
  float window_size_float_active = 0;
  float window_size_float_passive = 0;
  int senddelay = 500;

  double est_time = 0;
  double link_status = 0;
  double link_throughput = 0;
  /* Public interface for the congestion controller */
  /* You can change these if you prefer, but will need to change
     the call site as well (in sender.cc) */

  /* Default constructor */
  Controller( const bool debug );

  /* Get current window size, in datagrams */
  unsigned int window_size( void );

  /* A datagram was sent */
  void datagram_was_sent( const uint64_t sequence_number,
			  const uint64_t send_timestamp );

  /* An ack was received */
  void ack_received( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received );

  void ack_received_delay_threshold( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received );


  void ack_received_delay_threshold_varied_target( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received );

  void ack_received_prediction( const uint64_t sequence_number_acked,
		     const uint64_t send_timestamp_acked,
		     const uint64_t recv_timestamp_acked,
		     const uint64_t timestamp_ack_received );
 
 /* How long to wait (in milliseconds) if there are no acks
     before sending one more datagram */
  unsigned int timeout_ms( void );

  int send_counter=0;
  int recv_counter=0;
  uint64_t* send_time_list=NULL;
  uint64_t* recv_time_list=NULL;
  double* delay_list=NULL;

};

#endif
