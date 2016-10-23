#include <iostream>
#include <unistd.h>
#include <math.h>
#include <malloc.h>
#include <time.h>
#include "controller.hh"
#include "timestamp.hh"
#include "parameter.hh"

using namespace std;




/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), window_size_float(100.0)
{
}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  /* Default: fixed window size of 100 outstanding datagrams */
  unsigned int the_window_size = 5;

  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << the_window_size << endl;
  }

  return max((unsigned int)floor(window_size_float),(unsigned int)1);
  //return the_window_size;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  /* Default: take no action */
  if(this->send_time_list == NULL) 
  {
    this->send_time_list = (uint64_t*)malloc(sizeof(uint64_t)*65536*2);
    this->send_counter = 0;
  }

  //this->send_time_list[this->send_counter ++] = send_timestamp;
  this->send_time_list[this->send_counter ++] = timestamp_ms();
  //printf("%lu %lud\n",sequence_number, send_timestamp);

  usleep(senddelay); // sleep 500 microsecond


  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << endl;
  }
}

template<typename type> double Mean(type * ptr, int n)
{
  double ret = 0;
  double decay = 1.0;
  double decay_sum = 0;
  for(int i = n-1; i>=0; i--)
  {
    ret = ret + ptr[i] * decay;
    decay_sum+=decay;
    decay *= 0.97;
  }
  return ret/(decay_sum);
}

double Std(double* ptr, int n)
{
  double ret = 0;
  double mean = Mean(ptr,n);
  for(int i = 0; i<n; i++) ret = ret + (ptr[i] - mean)*(ptr[i] - mean);
  return sqrt(ret/n);
}

template<typename type> double dir(type* ptr, int n)
{
  double ret = 0;
  type mmax = 0;
  type mmin = 0;
  if(n <2) return 1000;

  for(int i = 0; i<n-1; i++)
  {
    ret = ret + ptr[i+1] - ptr[i];
    if(ptr[i+1]-ptr[i] > mmax) mmax = ptr[i+1] - ptr[i];
    if(ptr[i+1]-ptr[i] < mmin) mmin = ptr[i+1] - ptr[i];
  }
  return (ret)/(n-1);
}

double dir_exp(double* ptr, int n)
{
  double ret = 0;
  double w = 1.0;
  double sum = 0.0;
  double p = 1.2;

  for(int i = 0; i<n-1; i++)
  {
    ret = ret + (ptr[i+1]-ptr[i])*w;
    sum = sum + w;
    w = w * p;
  }
  return ret/sum;
}


double dir_power(double* ptr, int n)
{
  double ret = 0;
  double w = 1.0;
  double sum = 0.0;
  double p = 0.1;

  for(int i = 0; i<n-1; i++)
  {
    ret = ret + (ptr[i+1]-ptr[i])*w;
    sum = sum + w;
    w = w + p;
  }
  return ret/sum;
}


double throughput_est(uint64_t* t, int n)
{
  uint64_t delta = t[n-1] - t[0];
  //TODO prediction
  return n/((double)delta)*1000;
}






void Controller::ack_received_prediction( const uint64_t sequence_number_acked,
                               /* what sequence number was acknowledged */
                               const uint64_t send_timestamp_acked,
                               /* when the acknowledged datagram was sent (sender's clock) */
                               const uint64_t recv_timestamp_acked,
                               /* when the acknowledged datagram was received (receiver's clock)*/
                               const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  //static uint64_t old_delay = 0;
  uint64_t delay = timestamp_ack_received - send_timestamp_acked;

  static double* delay_list = NULL;
  static double* window_list = NULL;
  static uint64_t * ack_time_stamps = NULL;
  static int * time_window_count = NULL;


  static int counter = 0;




  if(delay_list == NULL)
  {
    delay_list = (double*)malloc(sizeof(double)*65536*2);
  }

  if(window_list == NULL)
  {
    window_list = (double*)malloc(sizeof(double)*65536*2);
  }

  if(ack_time_stamps == NULL)
  {
    ack_time_stamps = (uint64_t*)malloc(sizeof(uint64_t)*65536*2);
  }

  if(time_window_count == NULL)
  {
    time_window_count = (int*)malloc(sizeof(int)*65536*2);
  }




  #include "parameter.hh"
  int window = P_WINDOW;

  double w_old_avg = 0;
  double w_ins = 40;
  double w_target = 0;  
  double d_avg = 0;



  double mean_pf = 0;
  double mean_nf = 0;

  double feedback_pos = 0;
  double feedback_neg = 0;

  double feedback_avg = 0;



  double beta;

  static uint64_t TimeWindow = 20; // ms
  
  beta = P_BETA / 100.0; 
  
  
  delay_list[counter] = delay;
  window_list[counter] = w_ins;
  ack_time_stamps[counter] = timestamp_ack_received;
  time_window_count[counter] = 0;

    int cur_win = window;
    int cur_win_old = 0;

  if(counter>window*3)
  {

    cur_win = 0;
      
    while(cur_win_old < window*2 - 1)
    {
      if(timestamp_ack_received - ack_time_stamps[counter - cur_win-1] < TimeWindow)
        cur_win ++;
      if(timestamp_ack_received - ack_time_stamps[counter - cur_win_old -1] < TimeWindow * 2)
	cur_win_old ++;
      else break;
    }



    if(cur_win > window) cur_win = window;

    time_window_count[counter] = cur_win;





    w_old_avg = Mean(window_list + counter - (cur_win+1)*2+1, cur_win+1); // FIXME
    
    d_avg = Mean(delay_list + counter - cur_win, cur_win+1);

    double mean_tp = P_L0;

    mean_pf = min(max((mean_tp*2-d_avg)/mean_tp,0.0),2.0);
    mean_nf = 1.0 + max((d_avg-mean_tp)/mean_tp,-1.0);

    feedback_pos = mean_pf;
    feedback_neg = mean_nf;

    feedback_avg = -max(d_avg - P_L1, 0.0) * feedback_neg - min(d_avg - P_L2, 0.0) * feedback_pos; 

    w_target = max(w_old_avg + beta * feedback_avg , 0.0);

    w_ins = w_target;
  }
  delay_list[counter] = delay;
  window_list[counter] = w_ins;
  ack_time_stamps[counter] = timestamp_ack_received;
  counter ++;


  if(counter < 2) 
    printf("L1 %.2f L2 %.2f BETA %.2f\n",P_L1, P_L2, beta);

  window_size_float += min(w_ins - window_size_float,1.0);

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
         << " received ack for datagram " << sequence_number_acked
         << " (send @ time " << send_timestamp_acked
         << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
         << endl;
  }


}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  /* Default: take no action */
  if(this->recv_time_list == NULL)
  {
    this->recv_time_list = (uint64_t*)malloc(sizeof(uint64_t)*65536*2);
    this->recv_counter = 0;
  }

  if(this->delay_list == NULL)
  {
    this->delay_list = (double*)malloc(sizeof(double)*65536*2);
  }

  this->recv_time_list[this->recv_counter] = timestamp_ack_received;
  this->delay_list[this->recv_counter++] = timestamp_ack_received - send_timestamp_acked;

  /* Simple AIMD */
  /*
  static uint64_t last_sequence_number_acked = 0;

  if(sequence_number_acked != last_sequence_number_acked + 1)
  {
    window_size_float = max(window_size_float / 2, 1.0f); 
  }
  else
  {
    window_size_float = window_size_float + 1.0 / window_size_float;
  }

  last_sequence_number_acked = sequence_number_acked;

  printf("HST, %.2f, %lu\n",window_size_float, timestamp_ack_received - send_timestamp_acked);
  
  */
 
  ack_received_prediction(sequence_number_acked, send_timestamp_acked, recv_timestamp_acked, timestamp_ack_received);


  
  //if((int64_t)timestamp_ack_received - (int64_t)send_timestamp_acked > 80)
  //window_size_float /= 1.02;
  //else if(timestamp_ack_received - send_timestamp_acked > 100)
  //{
  //window_size_float /= 1.1;
  //}
  //else
  //window_size_float +=min(2.0/window_size_float,1.0);

  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return 50; /* timeout of 0.05 second */
}
