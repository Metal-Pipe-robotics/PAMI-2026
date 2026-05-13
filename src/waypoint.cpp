#include "waypoint.h"
#include <stdlib.h>

#define lock() 
#define unlock()

struct waypoint_t {
  struct waypoint_t *previous;
  struct Pose p;
  struct waypoint_t *next;
};

struct waypoint_list_t {
  struct waypoint_t *head;
  struct waypoint_t *tail;
  int n;
};

struct waypoint_list_t wl;
struct waypoint_list_t wpool;


/*====================================*/
/*          WAYPOINT TOOLS            */
/*====================================*/

static void _waypoint_add(struct waypoint_list_t *l, struct waypoint_t *w) {
  if (l->head == NULL) {
    l->head = w;
    l->tail = w;
    w->next = NULL;
    w->previous = NULL;
  } else {
    l->tail->next = w;
    w->previous = l->tail;
    w->next = NULL;
    l->tail = w;
  }
  l->n++;
}

static void _waypoint_add_front(struct waypoint_list_t *l, struct waypoint_t *w) {
  if (l->head == NULL) {
    l->head = w;
    l->tail = w;
    w->next = NULL;
    w->previous = NULL;
  } else {
    w->next = l->head;
    w->previous = NULL;
    l->head = w;
  }
  l->n++;
}

static struct waypoint_t *_waypoint_pop(struct waypoint_list_t *l) {
  if (l->head == NULL)
    return NULL;
  struct waypoint_t *w = l->head;
  if (w->next == NULL) {
    l->head = NULL;
    l->tail = NULL;
  } else {
    l->head = w->next;
  }
  w->next = NULL;
  w->previous = NULL;
  l->n--;
  return w;
}

/*===================================*/
/*          WAYPOINT POOL            */
/*===================================*/

static void waypoint_pool_give_back(struct waypoint_t *w) {
  _waypoint_add(&wpool, w);
}

void waypoint_pool_refill(int n) {
  struct waypoint_t *ws = (struct waypoint_t *) calloc(n, sizeof(waypoint_t));
  int i;
  for (i=0; i<n; i++) {
    waypoint_pool_give_back(ws + i);
  }
}

static struct waypoint_t *waypoint_pool_get_waypoint() {
  struct waypoint_t *w = _waypoint_pop(&wpool);
  if (w == NULL) {
    waypoint_pool_refill(10);
    return _waypoint_pop(&wpool);
  }
  return w;
}

static void waypoint_pool_init(int n) {
  wpool.head = NULL;
  wpool.tail = NULL;
  wpool.n = 0;
  struct waypoint_t *ws = (struct waypoint_t *) calloc(n, sizeof(waypoint_t));
  int i;
  for (i=0; i<n; i++) {
    waypoint_pool_give_back(ws + i);
  }
}


/*======================================*/
/*          WAYPOINT GESTION            */
/*======================================*/

void waypoint_add_pose(Pose p) {
  lock();
  struct waypoint_t *w = waypoint_pool_get_waypoint();
  w->p = p;
  _waypoint_add(&wl, w);
  unlock();
}

void waypoint_add_pose_front(Pose p) {
  lock();
  struct waypoint_t *w = waypoint_pool_get_waypoint();
  w->p = p;
  _waypoint_add_front(&wl, w);
  unlock();
}


struct Pose waypoint_pop_pose() {
  lock();
  struct waypoint_t *w = _waypoint_pop(&wl);
  if (w == NULL) {
    unlock();
    return (struct Pose) {-1, -1, -1, -1}; 
  }
  const struct Pose p = w->p;
  waypoint_pool_give_back(w);
  unlock();
  return p;
}

void waypoint_init_all() {
  lock();
  waypoint_pool_init(20);
  wl.head = NULL;
  wl.tail = NULL;
  wl.n = 0;
  unlock();
}

