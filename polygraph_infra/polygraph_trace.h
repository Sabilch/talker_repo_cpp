#ifndef POLYGRAPH_TRACE_H
#define POLYGRAPH_TRACE_H
 
/*!
 * \file polygraph_traces.h
 * \brief Implementation of a JSON Trace Event Format compatible execution trace (readable by the chrome::tracing tool)
 * \author Etienne Hamelin <etienne.hamelin@cea.fr>
 * \author Alexandre Berne <alexandre.berne@cea.fr> 
 * \version 2.0
 * \date 26/07/2023
 *
 * This file is generated automatically (contact A. Berne if errors)
 * This file is dedicated to the Polygraph Infrastructure code traces
 * (c) CEA List - DRT/LIST/DSCIN/LCYL
 *
 * \note 
 * The Trace Event Format is defined here:
 * https://docs.google.com/document/d/1CvAClvFfyA5R-PhYUmn5OOQtYMH4h6I0nSsKchNAySU/edit
 * 
 * Usage: open URL "chrome://tracing" in your chrome browser, click "load" and select your json trace file
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <assert.h>
#include <pthread.h>
#include <time.h>
#include <sys/syscall.h>

/** @brief Non POSIX portable, Linux-specific API */
#define gettid() syscall(__NR_gettid)

#define POLYTEF_ENABLED 1

/* Globals: */
FILE *Polytef_file;
unsigned long long Polytef_nb_events = -1;

/**
 * @brief Generate a timestamp for logging
 * Timestamps can be generated relative to the first to polytef_timestamp
 */
unsigned long long polytef_timestamp()
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  unsigned long long t = now.tv_sec * 1000000 + now.tv_nsec / 1000; // in microseconds

#ifdef POLYTEF_TIMESTAMP_RELATIVE
  static int initialized = 0;
  static unsigned long long t0 = 0;
  if (!initialized)
  {
    t0 = t; 
    initialized = 1;
  }
  return t - t0;
#else
  return t;
#endif
}

/**
 * @brief Open a trace file for logging
 * @param actorname name of the actor
 */
void polytef_init(const char *actorname)
{
#ifdef POLYTEF_ENABLED
  char filename[200];
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);
  snprintf(filename, sizeof(filename)-1, "/tmp/prod_cons/%s_%d-%02d-%02d_%02d-%02d-%02d.json", actorname, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec); /* global unique id for token */
  Polytef_file = fopen(filename, "w");
  assert(Polytef_file);

  fprintf(Polytef_file, "{\"traceEvents\": [\n");
  Polytef_nb_events = 0;
#endif
}

/**
 * @brief Add an event trace to the log
 * @param name name of the event
 * @param cat category
 * @param ph phase 
 * @param n_args number of additional key, value parameters
 * @param ... n_args times const char *key and const char *value, if you want to add a "args" : {"key1": "value1", "key2": "value2", ...}
 */
void polytef_add_event(const char *name, const char *cat, const char *ph, int n_args, ...)
{
#ifdef POLYTEF_ENABLED
  if (Polytef_file)
  {
    char buffer[1024];
    size_t len = 0;

    if (Polytef_nb_events)
    {
      len += sprintf(buffer + len, ",\n");
    }
    len += sprintf(buffer + len, 
        "{\"ts\": %llu, \"pid\": %u, \"tid\": \"%u\", \"ph\": \"%s\", \"cat\": \"%s\", \"name\": \"%s\"",
        polytef_timestamp(),
        (int)getpid(), 
        (int)gettid(),
        ph,
        cat, 
        name
        );

    va_list kv_list;
    va_start(kv_list, n_args);

    if (n_args)
    {
      len += sprintf(buffer + len, ", \"args\": {");
      for (int i = 0; i < n_args; i++)
      {
        if (i)
          len += sprintf(buffer + len, ", ");
        char *key = (char*)va_arg(kv_list, char*);
        char *value = (char*)va_arg(kv_list, char*);
        len += sprintf(buffer + len, "\"%s\": \"%s\"", key, value);
      }
      len += sprintf(buffer + len, "}");
    }
    len += sprintf(buffer + len, "}");

    fprintf(Polytef_file, "%s", buffer);
    ++Polytef_nb_events;
  }
#endif
}

/**
 * @brief Close the trace file
 */
void polytef_finalize()
{
#ifdef POLYTEF_ENABLED
  fprintf(Polytef_file, "\n]}");
  fclose(Polytef_file);
  Polytef_file = NULL;
#endif
}

/**
 * @brief Associate current thread id with a user-friendly name (optional)
 * @param name name of the thread
 */
void polytef_thread_name(const char* name)
{
#ifdef POLYTEF_ENABLED
  polytef_add_event("thread_name", "", "M", 1, "name", name);
#endif
}

/**
 * @brief Start a new job
 */
void polytef_job_start(const char *actor_name, unsigned long job_num)
{
#ifdef POLYTEF_ENABLED
  char buffer[12];
  snprintf(buffer, sizeof(buffer)-1, "%lu", job_num);
  polytef_add_event(actor_name, "actor", "B", 1, "job_num", buffer);
#endif
}

/**
 * @brief End of a job
 */
void polytef_job_finish()
{
#ifdef POLYTEF_ENABLED
  polytef_add_event("", "actor", "E", 0);
#endif
}

/**
 * @brief Send a token
 * @param channel_id a communication channel name
 * @param token_id token number
 * @note call polytef_channel_send() before actually sending the token
 */
void polytef_channel_send(const char* channel_id, unsigned long token_id)
{
#ifdef POLYTEF_ENABLED
  if (!Polytef_file)
    return;

  char id[50];
  snprintf(id, sizeof(id)-1, "%s_%ld", channel_id, token_id); /* global unique id for token */

  fprintf(Polytef_file, "%s{\"ts\": %llu, \"pid\": %u, \"tid\": \"%u\", \"ph\": \"s\", \"id\": \"%s\", \"cat\": \"msg\", \"name\": \"Message\", \"args\": {\"token\": \"%llu\"}}", 
          Polytef_nb_events ? ",\n" : "",
          polytef_timestamp(), 
          (unsigned int)getpid(),
          (unsigned int)gettid(),
          id, 
          (unsigned long long)token_id);

  ++Polytef_nb_events;
#endif
}

/**
 * @brief Receive a token
 * @param channel_id a communication channel name
 * @param token_id token number
 * @note Call polytef_channel_receive(...) after receiving the token, and before polytef_job_start() 
 */
void polytef_channel_receive(const char* channel_id, unsigned long token_id)
{
#ifdef POLYTEF_ENABLED
  if (!Polytef_file)
    return;

  char id[50];
  snprintf(id, sizeof(id)-1, "%s_%ld", channel_id, token_id); /* global unique id for token */

  fprintf(Polytef_file, "%s{\"ts\": %llu, \"pid\": %u, \"tid\": \"%u\", \"ph\": \"f\", \"id\": \"%s\", \"cat\": \"msg\", \"name\": \"Message\"}", 
          Polytef_nb_events ? ",\n" : "",
          polytef_timestamp(), 
          (unsigned int)getpid(),
          (unsigned int)gettid(),
          id);
  ++Polytef_nb_events;
#endif
}


#endif