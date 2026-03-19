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
 * we split the intersection into 4 conflict regions:
 *
 *   q[0] = north-west
 *   q[1] = north-east
 *   q[2] = south-east
 *   q[3] = south-west
 *
 * a car locks all regions it will pass through.
 * cars whose paths do not overlap can then cross simultaneously.
 */
static pthread_mutex_t quadrant_mutex[4];

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

// structure used to pass the lane information (side and direction) to each traffic light thread
typedef struct
{
  Side side;
  Direction direction;
} LightInfo;

// locks a single quadrant (conflict region) of the intersection and ensures that no other car can enter this region at the same time
static void lock_region(int region)
{
  pthread_mutex_lock(&quadrant_mutex[region]);
}

// unlocks a previously locked quadrant which allows other cars to use this part of the intersection
static void unlock_region(int region)
{
  pthread_mutex_unlock(&quadrant_mutex[region]);
}

// functions that lock and unlock all quadrants required for a cars path
static void lock_path(Side side, Direction direction);
static void unlock_path(Side side, Direction direction);

/*
 * manage_light(void* arg)
 *
 * A function that implements the behaviour of a traffic light
 */
static void* manage_light(void* arg)
{
  // TODO:
  // while it is not END_TIME yet, repeatedly:
  //  - wait for an arrival using the semaphore for this traffic light --> done
  //  - lock the right mutex(es) --> done
  //  - make the traffic light turn green --> done
  //  - sleep for CROSS_TIME seconds --> done
  //  - make the traffic light turn red --> done
  //  - unlock the right mutex(es) ---> done

  // extract the side and direction assigned to this traffic light
  LightInfo* light = (LightInfo*) arg;
  Side side = light->side;
  Direction direction = light->direction;

  int next_arrival = 0;  // index of the next car to process in this lane

  while (1) // repeatedly
  {
    // wait until a car arrives or until termination signal is sent
    sem_wait(&semaphores[side][direction]);

    // if this is a termination point exit after END_TIME
    if (curr_arrivals[side][direction][next_arrival].id == -1)
    {
      if (get_time_passed() >= END_TIME)
      {
        break;
      }
      continue;
    }
    // retrieve the next car for this lane
    Arrival arrival = curr_arrivals[side][direction][next_arrival];
    next_arrival++;

   // lock all required quadrants for this cars path
    lock_path(side, direction);

   //  allow the car to enter the intersection and print it 
    printf("traffic light %d %d turns green at time %d for car %d\n",
           side, direction, get_time_passed(), arrival.id);

    // simulate the time taken to cross the intersection
    sleep(CROSS_TIME);

    // car has passed so we turn the light red again
    printf("traffic light %d %d turns red at time %d\n",
           side, direction, get_time_passed());

   // release the quadrants so other cars can use them
    unlock_path(side, direction);
  }

  return(0);
}

// locks the quadrants needed for a car based on its side and direction.
// different movements require different sets of quadrants.
static void lock_path(Side side, Direction direction)
{
  /*
   * Conflict regions:
   * q0 = north-west
   * q1 = north-east
   * q2 = south-east
   * q3 = south-west
   *
   * Supported movements in this intersection:
   * NORTH:  STRAIGHT, RIGHT
   * SOUTH:  LEFT, STRAIGHT
   * EAST:   LEFT, STRAIGHT, RIGHT
   * WEST:   LEFT, RIGHT
   *
   * Locks are always taken in increasing order to avoid deadlock.
   */

  if (side == NORTH)
  {
    if (direction == STRAIGHT)        { lock_region(1); lock_region(2); }
    else if (direction == RIGHT)      { lock_region(0); }
    else                              { lock_region(0); lock_region(1); lock_region(2); lock_region(3); } /* unsupported */
  }
  else if (side == EAST)
  {
    if (direction == LEFT)            { lock_region(2); lock_region(3); }   /* east -> south */
    else if (direction == STRAIGHT)   { lock_region(0); lock_region(1); }   /* east -> west */
    else if (direction == RIGHT)      { lock_region(1); }                    /* east -> north */
  }
  else if (side == SOUTH)
  {
    if (direction == LEFT)            { lock_region(0); lock_region(3); }   /* south -> west */
    else if (direction == STRAIGHT)   { lock_region(1); lock_region(2); }   /* south -> north */
    else                              { lock_region(0); lock_region(1); lock_region(2); lock_region(3); } /* unsupported */
  }
  else if (side == WEST)
  {
    if (direction == LEFT)            { lock_region(0); lock_region(1); }   /* west -> north */
    else if (direction == RIGHT)      { lock_region(3); }                    /* west -> south */
    else                              { lock_region(0); lock_region(1); lock_region(2); lock_region(3); } /* unsupported */
  }
}

static void unlock_path(Side side, Direction direction)
{
  if (side == NORTH)
  {
    if (direction == STRAIGHT)        { unlock_region(2); unlock_region(1); }
    else if (direction == RIGHT)      { unlock_region(0); }
    else                              { unlock_region(3); unlock_region(2); unlock_region(1); unlock_region(0); }
  }
  else if (side == EAST)
  {
    if (direction == LEFT)            { unlock_region(3); unlock_region(2); }
    else if (direction == STRAIGHT)   { unlock_region(1); unlock_region(0); }
    else if (direction == RIGHT)      { unlock_region(1); }
  }
  else if (side == SOUTH)
  {
    if (direction == LEFT)            { unlock_region(3); unlock_region(0); }
    else if (direction == STRAIGHT)   { unlock_region(2); unlock_region(1); }
    else                              { unlock_region(3); unlock_region(2); unlock_region(1); unlock_region(0); }
  }
  else if (side == WEST)
  {
    if (direction == LEFT)            { unlock_region(1); unlock_region(0); }
    else if (direction == RIGHT)      { unlock_region(3); }
    else                              { unlock_region(3); unlock_region(2); unlock_region(1); unlock_region(0); }
  }
}
// all quadrants are unlocked in reverse order of locking to maintain consistency and avoid potential synchronization issues

int main(int argc, char * argv[])
{
  
  // mark every slot in curr_arrivals as empty by setting id = -1, this helps traffic-light threads detect termination/unused entries.
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      for (int k = 0; k < 20; k++)
      {
        curr_arrivals[i][j][k].id = -1;
      }
    }
  }

  // create semaphores to wait/signal for arrivals
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      sem_init(&semaphores[i][j], 0, 0);
    }
  }

  // initialize one mutex for each quadrant of the intersection 
  for (int i = 0; i < 4; i++)
  {
    pthread_mutex_init(&quadrant_mutex[i], NULL);
  }

  // start the timer
  start_time();

  // TODO: create a thread per traffic light that executes manage_light

  // arrays to store the thread ids and lane information of all traffic lights
  pthread_t light_threads[4][3];
  LightInfo light_info[4][3];

  // create one thread per traffic light 
  for (int side = 0; side < 4; side++)
  {
    for (int direction = 0; direction < 3; direction++)
    {
      light_info[side][direction].side = side;
      light_info[side][direction].direction = direction;

      pthread_create(&light_threads[side][direction],
                    NULL,
                    manage_light,
                    &light_info[side][direction]);
    }
  }

  // TODO: create a thread that executes supply_arrivals

  // create the supplier thread that generates arrivals and signals the correct lane
  pthread_t supplier_thread;   // stores supplier thread ID
  pthread_create(&supplier_thread, NULL, supply_arrivals, NULL);

  // TODO: wait for all threads to finish

  // wait until the supplier thread has finished adding all arrivals
  pthread_join(supplier_thread, NULL);

   // wait until the simulation reaches END_TIME
  sleep_until_arrival(END_TIME);

  // wake all traffic light threads so blocked threads can exit cleanly
  for (int side = 0; side < 4; side++)
  {
    for (int direction = 0; direction < 3; direction++)
    {
      sem_post(&semaphores[side][direction]);
    }
  }

 // wait for all traffic light threads to finish before ending the program
  for (int side = 0; side < 4; side++)
  {
    for (int direction = 0; direction < 3; direction++)
    {
      pthread_join(light_threads[side][direction], NULL);
    }
  }

  // destroy semaphores
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      sem_destroy(&semaphores[i][j]);
    }
  }

  for (int i = 0; i < 4; i++)
  {
    pthread_mutex_destroy(&quadrant_mutex[i]);
  }
}
