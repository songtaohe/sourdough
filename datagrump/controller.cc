#include <iostream>
#include <unistd.h>
#include <math.h>
#include <malloc.h>
#include <time.h>
#include "controller.hh"
#include "timestamp.hh"
#include "parameter.hh"

using namespace std;



pthread_t T;
void* controller_thread(void* context);

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ), window_size_float(100.0)
{
//pthread_create(&T, NULL, &controller_thread, this);
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


void Controller::ack_received_delay_threshold( const uint64_t sequence_number_acked,
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
  double alpha = 30.0;
  double target_delay = 80.0;  
  double recovery_speed = 5.0;

  if(delay > target_delay)
  {
    window_size_float = max(window_size_float / (2.0 - (1.0 + alpha)/(window_size_float + alpha)), 1.0);
    //window_size_float = max(window_size_float / ((target_delay + 2.0*(delay-target_delay))/(target_delay)),1.0);   

//    if(old_delay > target_delay && delay - old_delay > 10) window_size_float = 1.0;


  }
  else
  {
    window_size_float = window_size_float + recovery_speed / window_size_float;
  }
  
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }
}



double metricA(double* list, int n)
{
  double ret = 0;
  for(int i = 0; i<n-1; i++)
  {
    double jump = list[i+1] - list[i];
    ret += jump > 0 ? jump*jump : 0;
  }

  return ret/(n-1);
}


double metricB(double* list, int n)
{
  double ret = 0;
  for(int i = 0; i<n-1; i++)
  {
    double jump = list[i+1] - list[i];
    ret += jump < 0 ? jump*jump : 0;
  }

  return ret/(n-1);
}

void Controller::ack_received_delay_threshold_varied_target( const uint64_t sequence_number_acked,
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
  static double alpha = 100.0;
  static double target_delay = 80.0; 
  static double recovery_speed = 5.0;
  static double* delay_list = NULL;
  static double* window_list = NULL;
  static int counter = 0;


  static int state = 1; 
  int newstate = 1;
  //double table[3][2] = {{100.0, 10.0},{80.0, 5.0}, {60.0, 1.0 }};
  double table[3][2] = {{80.0, 5.0},{60.0, 3.0}, {40.0, 1.0 }};

  double mA = 0;
  double mB = 0;
  int mW = 32;
  

  if( counter > mW) mA = metricA(delay_list+(counter-mW),mW);
  if( counter > mW) mB = metricB(delay_list+(counter-mW),mW);


  if(mA >70) newstate = 2;

  if(state == 2 && mA < 40) newstate = 1;
  if(state == 0 && mA > 50) newstate = 1;

  if(mA < 30) newstate = 0;

  state = newstate;
  target_delay = table[state][0];
  recovery_speed = table[state][1];

  



  if(delay_list == NULL)
  {
    delay_list = (double*)malloc(sizeof(double)*65536*2);
  }

  if(window_list == NULL)
  {
    window_list = (double*)malloc(sizeof(double)*65536*2);
  }
 



  if(state>-1)
  { 
    if(delay > target_delay)
    { 
      window_size_float = max(window_size_float / (2.0 - (1.0 + alpha)/(window_size_float + alpha)), 1.0);
    }
    else
    { 
      window_size_float = window_size_float + recovery_speed / window_size_float;
    }
  }
  else
  {
    if(delay > target_delay)
    {
      window_size_float = max(window_size_float - 1.0, 1.0); 
    }
    else
    {
      window_size_float = window_size_float + 1.0;
    }

    //if(window_size_float > 70) window_size_float = 70;
  }

  if(state == 2 && window_size_float > 5) window_size_float = 5;
  //if(state == 1 && window_size_float > 50) window_size_float = 50;




  
  printf("HST, %.2f, %lu, %.2f, %.2f, %d\n",window_size_float, timestamp_ack_received - send_timestamp_acked, mA, mB, state*50);

  delay_list[counter] = delay;
  window_list[counter] = window_size_float;  
  counter ++;

  //if(counter < 5) 
  //printf("L1 %.2f L2 %.2f BETA %.2f\n",P_L1, P_L2, P_BETA);

 
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
         << " received ack for datagram " << sequence_number_acked
         << " (send @ time " << send_timestamp_acked
         << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
         << endl;
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

  //static int lastpeak = 0;



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


  //double TimeWarpBase = 5000;


  #include "parameter.hh"
  int window = P_WINDOW;

  double throughput = 0;

  double w_old_avg = 0;
  //double w_cur_avg = 0;
  double w_target = 0;
  double w_ins = 40;
  
  double d_dir = 0;
  double d_std = 0;
  double d_avg = 0;



  double mean_pf = 0;
  double mean_nf = 0;
  double std_pf = 0;
  double std_nf = 0;

  double feedback_pos = 0;
  double feedback_neg = 0;

  //double feedback_dir = 0;
  double feedback_avg = 0;


  //#include "parameter.hh"

  static double alpha = 0.01;
  double beta;
  static double lambda = 1.005;

  static uint64_t TimeWindow = 20; // ms
  
  beta = P_BETA / 100.0; 
  //beta = 0.035;
// printf("%.2f %.2f %.2f\n",beta, P_L1+1.0, P_L2+1.0);
  



  if(delay > 100) alpha = 0.01;
  else alpha = alpha * lambda;

  alpha = 0;

  //if(delay > 100) alpha = 0.4;
  //else alpha = 0.0;

  
  delay_list[counter] = delay;
  window_list[counter] = w_ins;
  ack_time_stamps[counter] = timestamp_ack_received;
  time_window_count[counter] = 0;

    int cur_win = window;
    int cur_win_old = 0;

  if(counter>window*3)
  {
    //int cur_win = window;
    //int cur_win_old = 0;

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

    //printf("Time Window %d\n",cur_win);
    //cur_win = 16;
    //cur_win = window;



    throughput = throughput_est(ack_time_stamps + counter -cur_win, cur_win);

    w_old_avg = Mean(window_list + counter - (cur_win+1)*2+1, cur_win+1); // FIXME
    //w_old_avg = Mean(window_list + counter - cur_win_old, cur_win_old - cur_win); // FIXME
    //w_old_avg = Mean(window_list + counter - cur_win-1, cur_win+1); // FIXME
    //w_cur_avg = Mean(window_list + counter - window, window);
    
    d_dir = dir(delay_list + counter - cur_win, cur_win+1);
    d_std = Std(delay_list + counter - cur_win, cur_win+1);
    d_avg = Mean(delay_list + counter - cur_win, cur_win+1);

    double mean_tp = P_L0 - alpha;

    mean_pf = min(max((mean_tp*2-d_avg)/mean_tp,0.0),2.0);
    mean_nf = 1.0 + max((d_avg-mean_tp)/mean_tp,-1.0);

    mean_pf = (mean_pf - 1.0)*1.0 + 1.0;
    mean_nf = (mean_nf - 1.0)*1.0 + 1.0;

    std_pf = min(max((40.0 - d_std) / 20.0, 0.0),10.0);
    std_nf = 1.0 + max((d_std - 20.0) / 20.0, 0.0);

    if(fabs(d_dir)<-0.01)
    {
      feedback_pos = mean_pf * std_pf;
      feedback_neg = mean_nf * std_nf;
    }
    else
    {
      feedback_pos = mean_pf;
      feedback_neg = mean_nf;
    }




    //feedback_dir = -max(d_dir,0.0) * feedback_neg - min(d_dir,0.0) * feedback_pos; 
    //feedback_avg = -max(d_avg - 68.0 + alpha, 0.0) * feedback_neg - min(d_avg - 72.0 - alpha, 0.0) * feedback_pos; 
    feedback_avg = -max(d_avg - P_L1 + alpha, 0.0) * feedback_neg - min(d_avg - P_L2 - alpha, 0.0) * feedback_pos; 


    
    //if(delay>80)
    //  w_target = min(max(w_old_avg + 0.0 * feedback_dir + beta * feedback_avg, 0.0), (double)window_size_float);
    //else
	double k = TimeWarpBase/throughput;
        k = min(max(k,0.9999),1.0001);
        //if(delay > 100) k *= 10;
      //w_target = max(w_old_avg + (0.0 * feedback_dir + beta * feedback_avg) * TimeWarpBase/throughput, 0.0);

      w_target = max(w_old_avg + (beta * feedback_avg)*k , 0.0);

      //if(beta * feedback_avg > 0) w_target = w_old_avg + beta * feedback_avg;
      //else w_target = w_old_avg / (1-beta*feedback_avg*0.2) * k;

      //w_target = max(window_size_float + (0.0 * feedback_dir + beta * feedback_avg)*k , 0.0);
     if(debug_) printf("%lf %lf\n",w_target,w_old_avg);
      //if(delay < 70 && throughput > 100) w_target = 0.08 * throughput;
    //w_ins = max(w_target * (window + 1) - w_cur_avg * window, 0.0);

     //if(d_avg>80) w_ins = window_size_float / 1.02;
     //else w_ins = window_size_float + min(2.0/window_size_float, 1.0);

 
     //if(feedback_avg < 0) w_ins = window_size_float / (1.0 - 0.001 * feedback_avg);
     //else w_ins = window_size_float + min((0.02*feedback_avg)/window_size_float, 1.0);

 

         

   w_ins = w_target;
   //if(delay > 80) w_ins /=1.02;







    //Count Prediction
    int _n[16];
    int _flag = 0;
    int _c = counter;
    _n[0] = time_window_count[_c];
    for(int i = 0; i<6; i++)
    {
        //printf("%d %d \n",_c,_n[i]);
	if( _c - _n[i] -1 > window*2)
	{
	  _n[i+1] = time_window_count[_c - _n[i]-1];
	  _c = _c - _n[i]-1;
	}
        else
	{
	  _flag = 1;
          break;
	}

    }


    
    if(_flag == 0)
    {
       double trend = dir<int>(_n,6);
       double avg = Mean(_n,6);
       double predict = 0;

       predict = avg - 4*trend;
       //printf("%d %d %d %d %d %d %3.1lf\n",_n[5], _n[4], _n[3], _n[2], _n[1], _n[0], predict);
       if(predict < 3 && trend > 0 )
       {
         //printf("Alert predict %lf\n",predict);        
         //w_ins = max(min(w_ins*0.95 , w_ins -0.5),0.0);
       }
    }

  }




  //printf("T %.2f\n",throughput);

  //printf("HST, %.2f, %lu, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",window_size_float, timestamp_ack_received - send_timestamp_acked, w_old_avg, w_cur_avg, w_target, w_ins,d_dir, d_std, d_avg, feedback_dir, feedback_avg,alpha);





  //printf("T %.2f\n",throughput);

  //printf("HST, %.2f, %lu, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",window_size_float, timestamp_ack_received - send_timestamp_acked, w_old_avg, w_cur_avg, w_target, w_ins,d_dir, d_std, d_avg, feedback_dir, feedback_avg,alpha);



  delay_list[counter] = delay;
  window_list[counter] = w_ins;
  ack_time_stamps[counter] = timestamp_ack_received;
  counter ++;


  if(counter < 5) 
      printf("L1 %.2f L2 %.2f BETA %.2f\n",P_L1, P_L2, beta);
  //if(delay > 180) w_ins = 0;
  //if(d_dir > 3) w_ins = 0;

  //window_size_float_active = w_ins;
  //printf("Delay %15lu Delta %15.6lf Delta %15.6lf  Feedback AVG %15.6lf\n", delay, w_ins - window_size_float, w_ins, feedback_avg);

  window_size_float += min(w_ins - window_size_float,1.0);


  // Dirty Code
  //uint64_t cur_time = ack_time_stamps[counter-1] - ack_time_stamps[0];
  //static int idle_state = 0;

/*
  if(cur_time > 60000 && cur_time < 70000)
  {
    idle_state = 1;
     window_size_float = 10000; // a lot but with delay
     senddelay = 40* 1000;// 40ms 
  }
  else
  {
     if(idle_state == 1) window_size_float = 40;
     idle_state = 0;
     senddelay = 500;
  }
*/


  //printf("%lu\n",cur_time);

 
  //printf("%lf\n",throughput);

  //if(throughput < 1000)
  //  window_size_float = min((double)window_size_float, throughput * 0.8);
  
  
 // if(cur_win < 32) 
 //    window_size_float = min((double)window_size_float, 64.0);


  /*
  double merger_factor = 0.0;
  double min_w = min(w_ins, (double)window_size_float_passive);
  double max_w = max(w_ins, (double)window_size_float_passive);



  if( link_status < 50) merger_factor = link_status / 50.0;
  merger_factor = 0.0;
  if( link_status > 2500) merger_factor = min((link_status - 2500)/1000.0, 1.0); 

  if(est_time < 10) merger_factor = 1.0;
  if(est_time <95 && est_time >85) merger_factor = max(0.5,merger_factor);



  window_size_float = min_w * merger_factor + max_w * (1.0 - merger_factor);
*/

/*
  if(link_throughput < 200.0) window_size_float *= 0.95;
  if(link_throughput < 100.0) window_size_float *= 0.9;
  if(link_throughput < 50.0) window_size_float *= 0.8;
*/
  //if(link_status > 3000) window_size_float = min(w_ins, (double)window_size_float_passive);
  //else if(link_status < 50) window_size_float = min(w_ins, (double)window_size_float_passive);
  //else window_size_float = max(w_ins, (double)window_size_float_passive);
  


  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
         << " received ack for datagram " << sequence_number_acked
         << " (send @ time " << send_timestamp_acked
         << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
         << endl;
  }


}




#define ML_WIN 8
#define ML_P 4

double __ML__(double * input, int flush, int* flag)
{
   static FILE *fp = NULL;
   static double W[ML_WIN * ML_P + 1];
   static double X[ML_WIN * ML_P + 1];
   static int n = 0;
   static int ptr = 0;
   static int init = 0;
   double output = 0;
   *flag = 0;
   if(fp == NULL)
   {
     fp = fopen("Para.txt","rt");
     for(int i = 0; i< ML_WIN * ML_P + 1; i++)
     {
        int ret = 0;
	ret = fscanf(fp,"%lf",W+i);
        printf("%lf %d\n",W[i],ret);
     }
     fclose(fp);
     init = 1;
   } 

   if(init)
   {
     if(flush) n = 0;
     else
     {
       for(int i = 0; i < ML_P; i++)
       {
         X[ptr*ML_P+i] = input[i];
       } 
      ptr++;
      if(ptr >= ML_WIN) ptr = 0;
      n=n+1;
  
      if(n>= ML_WIN)
      {
         output = W[ML_WIN * ML_P];
         for(int i = 0; i< ML_P; i++)
         {
           for(int j = 0; j<ML_WIN; j++)
             output += W[j + i*ML_WIN] * X[((ptr+ML_WIN-j-1) % ML_WIN) * ML_P + i];
             //output += W[j + i*ML_WIN] * X[((ptr-j-1) % ML_WIN) * ML_P + i];
         }
         *flag = 1;
      }
         
     }
   }

   
   return output;


}






void* controller_thread(void* context)
{
  Controller* C = (Controller*)context;
  static int old_recv_counter = 0;
  static int old_recv_counter_2 = 0;
  static int old_recv_counter_3 = 0;
  static int state = 0;
  static int n = 0;
  double windowsize = 40;
  double old_thr = 0;
  //static double old_dly = 0;
  //static int old_send_counter = 0;
  int scale = 200;
  while(1)
  {
    //TODO
    C->est_time += 200*100/1000000.00;
    if(C->recv_counter > 64)
    {
      n = n + 1;
      int recv_counter = C->recv_counter - 1; // Remove "Minus 1"
      //int send_counter = C->send_counter - 1;
      double thr1 = 0;

      //double F_dly_std = 0;
      //double F_dly_avg = 0;
      //double F_dly_dir = 0;

      //double ML_X[ML_P];
      //double ML_result = 0;
      //int flag = 0;

      //double ML_error = 1.0;

      if(recv_counter - old_recv_counter_3 <=2)
      {
        state += 1; // outage;
       // __ML__(ML_X,1,&flag);
      }
      else
      {
        thr1 = throughput_est((C->recv_time_list)+old_recv_counter_3, recv_counter - old_recv_counter_3);
        double thr2 = (recv_counter - old_recv_counter_3)/(scale*0.000001*300);

        if((thr1 - thr2) / thr2 > 1.5 ) thr1 = thr2; // Fix the potential mis-estimation

	//F_dly_std = Std((C->delay_list)+old_recv_counter_3, recv_counter - old_recv_counter_3);
	//F_dly_avg = Mean((C->delay_list)+old_recv_counter_3, recv_counter - old_recv_counter_3);
	//F_dly_dir = dir((C->delay_list)+old_recv_counter_3, recv_counter - old_recv_counter_3);

        state = 0; 

        //ML_X[0] = thr1;
        //ML_X[1] = F_dly_avg;
        //ML_X[2] = max(F_dly_dir,0.0);
        //ML_X[3] = min(F_dly_dir,0.0);

        //ML_result = __ML__(ML_X,0,&flag);
        


      }



      //ML_error = ML_error * 0.5 + fabs(F_dly_avg - old_dly)*0.5;
      //printf("%4d %4d %4d %15.6lf  %15.6lf  %15.6lf  %15.6lf %15.6lf %15.6lf %15.6lf \n",n, state, recv_counter - old_recv_counter_3, thr1, F_dly_avg, F_dly_std, F_dly_dir, ML_result, fabs(F_dly_avg - old_dly), ML_error);
      //ML_error = ML_error * 0.5 + fabs(F_dly_avg - old_dly)*0.5
      //old_dly = ML_result;




      if(state > 0) // outage;
      {
         C->link_status /= 2;
         //if(state == 3 || state == 7 || state == 12 || state == 19 || state == 31 || state == 41 || state == 51 || state == 64 || state == 78 || state > 95) windowsize = 6;
         //if(state == 3 || state == 7 || state == 12 || state == 19 || state == 31 || state >48) windowsize = 6;
         //if(state % 3 == 0) windowsize = 6;
         //else windowsize = 0;  
         windowsize = 0;
      }
      else
      {
        C->link_status += scale/10.0;
        if(old_thr >= 1.0)
        {
          if(old_thr > thr1) windowsize = 0.08 * max(thr1*1.618 - old_thr*0.618, 0.0); // prediction
          else windowsize = 0.08 * max(thr1*1.382 - old_thr*0.382, 0.0); // prediction
        }
        else
          windowsize = 0.08 * thr1;
      }

      // ML   avoid high latency
      //if(ML_error < 10)
      //{
      //  if(ML_result > 90) windowsize /= 1.3;
      //  if(ML_result > 100) windowsize /= 1.5;
      //  if(ML_result > 120) windowsize /= 1.7;
      //}
      /*
      if(F_dly_avg > 130) windowsize /= 1.5;
      if(F_dly_avg > 140) windowsize /= 1.5;
      if(F_dly_avg > 160) windowsize /= 1.5;
      */


      old_thr = thr1;

      C->link_throughput = C->link_throughput * 0.8 + thr1 * 0.2;

      //printf("%4d Throughput %9.2f AVG throughput %6.1f Based on %3d packets  State %2d   Windowsize %6.2f  LinkStatus %6.2f \n",n, thr1, C->link_throughput/100.0, recv_counter - old_recv_counter_3, state, windowsize, C->link_status);
      
      

      old_recv_counter_3 = old_recv_counter_2;
      old_recv_counter_2 = old_recv_counter;
      old_recv_counter = recv_counter;
    }   

//    int mask = 1;

    //Smooth the windowsize
    /*
    if(windowsize > C->window_size_float + 1.0)
    {
      double delta = windowsize - C->window_size_float;
      for(int i = 0; i< 5; i++)
      {
        if(mask) C->window_size_float += delta/5.0;
        usleep(10*scale); // Increase Every 10 ms
      }
      usleep(40*scale);// overhead
    }
    else
    {
      if(mask) C->window_size_float = windowsize;    
*/
      C->window_size_float = windowsize;
      usleep(100*scale); // 100ms
  //  }
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

  /* Delay Threshold */
  

  //ack_received_delay_threshold(sequence_number_acked, send_timestamp_acked, recv_timestamp_acked, timestamp_ack_received);
  //ack_received_delay_threshold_varied_target(sequence_number_acked, send_timestamp_acked, recv_timestamp_acked, timestamp_ack_received);
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
  return 50; /* timeout of one second */
}
