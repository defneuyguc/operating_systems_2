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
 * region_mutex[]
 *
 * Mutexes for the conflict regions inside the intersection
 */
static pthread_mutex_t region_mutex[9];

/*
 * LightArgs
 *
 * Stores the side and direction assigned to one traffic light thread
 */
typedef struct {
  Side side;
  Direction direction;
} LightArgs;

/*
 * get_path_regions(side, direction, regions, count)
 *
 * Determines which conflict regions are used by a car path.
 *
 * Region layout:
 *   0 = north-west corner
 *   1 = north-east corner
 *   2 = south-east corner
 *   3 = south-west corner
 *   4 = north/south upper strip
 *   5 = east/west right strip
 *   6 = north/south lower strip
 *   7 = east/west left strip
 *   8 = center crossing region
 */
static void get_path_regions(Side side, Direction direction, int regions[], int *count)
{
  *count = 0;

  if (side == NORTH)
  {
    if (direction == RIGHT)
    {
      regions[0] = 1;
      *count = 1;
    }
    else if (direction == STRAIGHT)
    {
      regions[0] = 4;
      regions[1] = 8;
      *count = 2;
    }
    else 
    {
      regions[0] = 4;
      regions[1] = 5;
      regions[2] = 8;
      *count = 3;
    }
  }
  else if (side == EAST)
  {
    if (direction == RIGHT)
    {
      regions[0] = 2;
      *count = 1;
    }
    else if (direction == STRAIGHT)
    {
      regions[0] = 5;
      regions[1] = 8;
      *count = 2;
    }
    else 
    {
      regions[0] = 5;
      regions[1] = 6;
      regions[2] = 8;
      *count = 3;
    }
  }
  else if (side == SOUTH)
  {
    if (direction == RIGHT)
    {
      regions[0] = 3;
      *count = 1;
    }
    else if (direction == STRAIGHT)
    {
      regions[0] = 6;
      regions[1] = 8;
      *count = 2;
    }
    else
    {
      regions[0] = 6;
      regions[1] = 7;
      regions[2] = 8;
      *count = 3;
    }
  }
  else if (side == WEST)
  {
    if (direction == RIGHT)
    {
      regions[0] = 0;
      *count = 1;
    }
    else if (direction == STRAIGHT)
    {
      regions[0] = 7;
      regions[1] = 8;
      *count = 2;
    }
    else 
    {
      regions[0] = 7;
      regions[1] = 4;
      regions[2] = 8;
      *count = 3;
    }
  }
}

/*
 * lock_regions(side, direction)
 *
 * Locks the conflict regions needed by this car path
 * Mutexes are always locked in increasing order
 */
static void lock_regions(Side side, Direction direction)
{
  int regions[3];
  int count = 0;

  get_path_regions(side, direction, regions, &count);

  for (int i = 0; i < count; i++)
  {
    pthread_mutex_lock(&region_mutex[regions[i]]);
  }
}

/*
 * unlock_regions(side, direction)
 *
 * Unlocks the conflict regions needed by this car path
 * Mutexes are unlocked in reverse order
 */
static void unlock_regions(Side side, Direction direction)
{
  int regions[3];
  int count = 0;

  get_path_regions(side, direction, regions, &count);

  for (int i = count - 1; i >= 0; i--)
  {
    pthread_mutex_unlock(&region_mutex[regions[i]]);
  }
}

/*
 * supply_arrivals()
 *
 * A function for supplying arrivals to the intersection
 * This should be executed by a separate thread
 */
static void* supply_arrivals(void *arg)
{
  (void) arg;

  int num_curr_arrivals[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

  // for every arrival in the list
  for (int i = 0; i < (int)(sizeof(input_arrivals) / sizeof(Arrival)); i++)
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
  return 0;
}

/*
 * manage_light(void* arg)
 *
 * A function that implements the behaviour of a traffic light
 */
static void* manage_light(void* arg)
{
  // get the side and direction of the traffic light thread
  LightArgs *args = (LightArgs *) arg;
  Side side = args->side;
  Direction direction = args->direction;

  int next_arrival = 0; // index of the next car

  // keep processing cars for this lane
  while (1)
  {
    // wait until a car arrives for this traffic light
    sem_wait(&semaphores[side][direction]);

    // get the next arrived car for this lane
    Arrival arrival = curr_arrivals[side][direction][next_arrival];
    next_arrival++;

    // stop after the termination marker for this lane
    if (arrival.id == -1)
    {
      break;
    }

    // lock all intersection regions needed for this car's path
    lock_regions(arrival.side, arrival.direction);

    // allow the car to enter the intersection
    printf("traffic light %d %d turns green at time %d for car %d\n",
           arrival.side, arrival.direction, get_time_passed(), arrival.id);

    // let the car cross the intersection
    sleep(CROSS_TIME);

    // turn the traffic light red again
    printf("traffic light %d %d turns red at time %d\n",
           arrival.side, arrival.direction, get_time_passed());

    // unlock the intersection regions used by the car
    unlock_regions(arrival.side, arrival.direction);
  }

  return 0;
}

int main(int argc, char * argv[])
{
  (void) argc;
  (void) argv;

  pthread_t light_threads[4][3];
  LightArgs light_args[4][3];
  pthread_t supplier_thread;

  // create semaphores to wait/signal for arrivals
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      sem_init(&semaphores[i][j], 0, 0);
    }
  }

  // create mutexes for the conflict regions
  for (int i = 0; i < 9; i++)
  {
    pthread_mutex_init(&region_mutex[i], NULL);
  }

  // start the timer
  start_time();

  // create a thread per traffic light that executes manage_light
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      light_args[i][j].side = (Side) i;
      light_args[i][j].direction = (Direction) j;
      pthread_create(&light_threads[i][j], NULL, manage_light, &light_args[i][j]);
    }
  }

  // create a thread that executes supply_arrivals
  pthread_create(&supplier_thread, NULL, supply_arrivals, NULL);

  // wait for the supplier thread to finish
  pthread_join(supplier_thread, NULL);

  // wait until END_TIME before terminating the traffic light threads
  if (get_time_passed() < END_TIME)
  {
    sleep(END_TIME - get_time_passed());
  }

  // count how many arrivals each lane received
  int lane_counts[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

  for (int i = 0; i < (int)(sizeof(input_arrivals) / sizeof(Arrival)); i++)
  {
    lane_counts[input_arrivals[i].side][input_arrivals[i].direction]++;
  }

  // add one termination marker per lane and wake each traffic light once more
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      curr_arrivals[i][j][lane_counts[i][j]].id = -1;
      curr_arrivals[i][j][lane_counts[i][j]].side = (Side) i;
      curr_arrivals[i][j][lane_counts[i][j]].direction = (Direction) j;
      curr_arrivals[i][j][lane_counts[i][j]].time = END_TIME;

      sem_post(&semaphores[i][j]);
    }
  }

  // wait until all traffic light threads have finished
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      pthread_join(light_threads[i][j], NULL);
    }
  }

  // destroy mutexes
  for (int i = 0; i < 9; i++)
  {
    pthread_mutex_destroy(&region_mutex[i]);
  }

  // destroy semaphores
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      sem_destroy(&semaphores[i][j]);
    }
  }

  return 0;
}