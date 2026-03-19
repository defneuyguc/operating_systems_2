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
 * We split the intersection into 4 conflict regions:
 *
 *   q[0] = north-west
 *   q[1] = north-east
 *   q[2] = south-east
 *   q[3] = south-west
 *
 * A car locks all regions it will pass through.
 * Cars whose paths do not overlap can then cross simultaneously.
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

typedef struct
{
  Side side;
  Direction direction;
} LightInfo;

static void lock_region(int region)
{
  pthread_mutex_lock(&quadrant_mutex[region]);
}

static void unlock_region(int region)
{
  pthread_mutex_unlock(&quadrant_mutex[region]);
}

/* 
 * These two functions will be filled in when we do the mutex mapping for 2b.
 * For now, manage_light() will call them.
 */
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
  //  - wait for an arrival using the semaphore for this traffic light
  //  - lock the right mutex(es)
  //  - make the traffic light turn green
  //  - sleep for CROSS_TIME seconds
  //  - make the traffic light turn red
  //  - unlock the right mutex(es)

  LightInfo* light = (LightInfo*) arg;
  Side side = light->side;
  Direction direction = light->direction;

  int next_arrival = 0;

  while (1)
  {
    /* wait until a car arrives for this lane, or until main wakes us up to stop */
    sem_wait(&semaphores[side][direction]);

    /*
     * If there is no real car stored at this position, then this wake-up is only
     * the termination signal. In that case, stop after END_TIME.
     */
    if (curr_arrivals[side][direction][next_arrival].id == -1)
    {
      if (get_time_passed() >= END_TIME)
      {
        break;
      }
      continue;
    }

    Arrival arrival = curr_arrivals[side][direction][next_arrival];
    next_arrival++;

    /* claim the required part(s) of the intersection */
    lock_path(side, direction);

    printf("traffic light %d %d turns green at time %d for car %d\n",
           side, direction, get_time_passed(), arrival.id);

    sleep(CROSS_TIME);

    printf("traffic light %d %d turns red at time %d\n",
           side, direction, get_time_passed());

    /* release the required part(s) of the intersection */
    unlock_path(side, direction);
  }

  return(0);
}

static void lock_path(Side side, Direction direction)
{
  /*
   * Fixed global order: always lock regions in increasing order.
   * This follows the lecture rule for deadlock prevention.
   */

  if (side == NORTH)
  {
    if (direction == RIGHT)      { lock_region(1); }
    else if (direction == STRAIGHT) { lock_region(1); lock_region(2); }
    else                        { lock_region(1); lock_region(2); lock_region(3); }
  }
  else if (side == EAST)
  {
    if (direction == RIGHT)      { lock_region(2); }
    else if (direction == STRAIGHT) { lock_region(2); lock_region(3); }
    else                        { lock_region(0); lock_region(2); lock_region(3); }
  }
  else if (side == SOUTH)
  {
    if (direction == RIGHT)      { lock_region(3); }
    else if (direction == STRAIGHT) { lock_region(0); lock_region(3); }
    else                        { lock_region(0); lock_region(1); lock_region(3); }
  }
  else if (side == WEST)
  {
    if (direction == RIGHT)      { lock_region(0); }
    else if (direction == STRAIGHT) { lock_region(0); lock_region(1); }
    else                        { lock_region(0); lock_region(1); lock_region(2); }
  }
}

static void unlock_path(Side side, Direction direction)
{
  /*
   * Unlock the same regions that were locked for this path.
   */

  if (side == NORTH)
  {
    if (direction == RIGHT)      { unlock_region(1); }
    else if (direction == STRAIGHT) { unlock_region(2); unlock_region(1); }
    else                        { unlock_region(3); unlock_region(2); unlock_region(1); }
  }
  else if (side == EAST)
  {
    if (direction == RIGHT)      { unlock_region(2); }
    else if (direction == STRAIGHT) { unlock_region(3); unlock_region(2); }
    else                        { unlock_region(3); unlock_region(2); unlock_region(0); }
  }
  else if (side == SOUTH)
  {
    if (direction == RIGHT)      { unlock_region(3); }
    else if (direction == STRAIGHT) { unlock_region(3); unlock_region(0); }
    else                        { unlock_region(3); unlock_region(1); unlock_region(0); }
  }
  else if (side == WEST)
  {
    if (direction == RIGHT)      { unlock_region(0); }
    else if (direction == STRAIGHT) { unlock_region(1); unlock_region(0); }
    else                        { unlock_region(2); unlock_region(1); unlock_region(0); }
  }
}

int main(int argc, char * argv[])
{
  
  /* mark all curr_arrivals slots as empty */
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

  for (int i = 0; i < 4; i++)
  {
    pthread_mutex_init(&quadrant_mutex[i], NULL);
  }

  // start the timer
  start_time();

  // TODO: create a thread per traffic light that executes manage_light

  pthread_t light_threads[4][3];
  LightInfo light_info[4][3];

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
  pthread_t supplier_thread;
  pthread_create(&supplier_thread, NULL, supply_arrivals, NULL);

  // TODO: wait for all threads to finish
  pthread_join(supplier_thread, NULL);

  /* wake up all traffic-light threads so they can exit cleanly */
  for (int side = 0; side < 4; side++)
  {
    for (int direction = 0; direction < 3; direction++)
    {
      sem_post(&semaphores[side][direction]);
    }
  }

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
