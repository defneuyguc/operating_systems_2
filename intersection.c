#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>

#include "arrivals.h"
#include "intersection_time.h"
#include "input.h"

/* 
 * curr_arrivals[][][]
 *
 * A 3D array that stores the arrivals that have occurred
 * The first two indices determine the entry lane: first index is Side, second index is Direction
 * curr_arrivals[s][d] returns an array of all arrivals for the entry lane on side s for direction d,
 *   ordered in the same order as they arrived
 */
static Arrival curr_arrivals[4][3][20];

/*
 * semaphores[][]
 *
 * A 2D array that defines a semaphore for each entry lane,
 *   which are used to signal the corresponding traffic light that a car has arrived
 * The two indices determine the entry lane: first index is Side, second index is Direction
 */
static sem_t semaphores[4][3];

/*
 * supply_arrivals()
 *
 * A function for supplying arrivals to the intersection
 * This should be executed by a separate thread
 */
static void* supply_arrivals()
{
  int num_curr_arrivals[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

  // for every arrival in the list
  for (int i = 0; i < sizeof(input_arrivals)/sizeof(Arrival); i++)
  {
    // get the next arrival in the list
    Arrival arrival = input_arrivals[i];
    // wait until this arrival is supposed to arrive
    sleep_until_arrival(arrival.time);
    // store the new arrival in curr_arrivals
    curr_arrivals[arrival.side][arrival.direction][num_curr_arrivals[arrival.side][arrival.direction]] = arrival;
    num_curr_arrivals[arrival.side][arrival.direction] += 1;
    // increment the semaphore for the traffic light that the arrival is for
    sem_post(&semaphores[arrival.side][arrival.direction]);
  }

  return(0);
}

// each traffic light manages exactly one lane so it must know which side and which direction it belongs to
typedef struct {
  Side side;
  Direction direction;
} LightArgs;

static pthread_mutex_t intersection_mutex; // single shared mutex for the whole intersection

/*
 * manage_light(void* arg)
 *
 * A function that implements the behaviour of a traffic light
 */
static void* manage_light(void* arg)
{
  // TODO:
  // while it is not END_TIME yet, repeatedly: --> done
  //  - wait for an arrival using the semaphore for this traffic light --> done
  //  - lock the right mutex(es) --> done
  //  - make the traffic light turn green -> done
  //  - sleep for CROSS_TIME seconds --> done
  //  - make the traffic light turn red --> done
  //  - unlock the right mutex(es) --> done

  LightArgs *args = (LightArgs *) arg;
  Side side = args->side;  // extract the side of the intersection that the thread controls
  Direction direction = args->direction; // extract the direction this traffic light allows cars to go

  int next_arrival = 0; // this keeps trck of which car is in the list

  while (1) // the assignment says traffic lights should run until END_TIME.
  // so we repeatedly wait for cars and let them pass until the simulation ends.
  {
    // wait until a car arrives for this lane
    // supply_arrivals() will call sem_post() when a car arrives
    // sem_wait() blocks the thread (no busy waiting) until a car exists
    sem_wait(&semaphores[side][direction]);

    // if we were only woken up to terminate stop here
    if (get_time_passed() >= END_TIME)
    {
      break;
    }

    // get the next car for this lane
    Arrival arrival = curr_arrivals[side][direction][next_arrival];
    next_arrival++; // move to the next car for the next iteration

    // only one car in the intersection at a time (basic solution) therefore all traffic lights must share one mutex
    pthread_mutex_lock(&intersection_mutex);

    // print information that the light turns green 
     printf("traffic light %d %d turns green at time %d for car %d\n",
           side, direction, get_time_passed(), arrival.id);
    // once the car enters the intersection, the thread must sleep for CROSS_TIME seconds while the car crosses
    // during this time no other car can enter the intersection.
     sleep(CROSS_TIME);

    // after the car has crossed, the light turns red again
    printf("traffic light %d %d turns red at time %d\n",
           side, direction, get_time_passed());

    // release the intersection so another traffic light can allow a car - wakes up another thread waiting for the mutex
    pthread_mutex_unlock(&intersection_mutex);
  }

  return(0);
}


int main(int argc, char * argv[])
{
  // create semaphores to wait/signal for arrivals
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      sem_init(&semaphores[i][j], 0, 0);
    }
  }

  // start the timer
  start_time();

  // TODO: create a thread per traffic light that executes manage_light
  
    // initialize the mutex that represents the intersection - this will enforce mutual exclusion
    pthread_mutex_init(&intersection_mutex, NULL);

    pthread_t light_threads[4][3]; // array to store the thread ids of all traffic light threads
    LightArgs light_args[4][3]; // matching array of traffic lights
    pthread_t supplier_thread; // generates car arrivals over time and signals the semaphores

      for (int i = 0; i < 4; i++) //  loop over all possible sides of the intersection
  {
    for (int j = 0; j < 3; j++) // for each side, loop over all possible directions a car can go 
    // 0=left 1=straight 2=right

    {
      light_args[i][j].side = (Side) i; // store which side this thread controls
      light_args[i][j].direction = (Direction) j; // store which direction this traffic-light thread controls
      // create a traffic-light thread that runs manage_light(). 
      // we pass the lane information (side and direction) so the thread knows which traffic light it controls
      pthread_create(&light_threads[i][j], NULL, manage_light, &light_args[i][j]);
    }
  }

  // TODO: create a thread that executes supply_arrivals
   pthread_create(&supplier_thread, NULL, supply_arrivals, NULL);

  // TODO: wait for all threads to finish

  // wait for the supplier thread to finish generating all car arrivals
    pthread_join(supplier_thread, NULL);
 
    // wait until the simulation reaches END_TIME before shutting down the lights
    if (get_time_passed() < END_TIME)
    {
      sleep(END_TIME - get_time_passed());
    }

    // wake up all traffic light threads that may be blocked in sem_wait()
     for (int i = 0; i < 4; i++)
     {
      for (int j = 0; j < 3; j++)
      {
         sem_post(&semaphores[i][j]);
         }
        }
        
    // wait for all traffic light threads to finish execution.
     for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          pthread_join(light_threads[i][j], NULL);
        }
      }
    // destroy the mutex since its no longer needed
    pthread_mutex_destroy(&intersection_mutex);

  // destroy semaphores
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      sem_destroy(&semaphores[i][j]);
    }
  }
}
