/* SPDX-License-Identifier: MIT */
/*
 * KVStore - Simple key-value store library
 * Copyright (C) 2025 Vítor Santos
 */

#include "kvstore.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <pthread.h>

static pthread_mutex_t kv_mutex = PTHREAD_MUTEX_INITIALIZER;

static uint32_t kv_store_hash(const char *key);
void kv_node_free(struct kv_node *node);
struct kv_node *kv_store_lookup(const struct kv_store *store, const char *key);

/**
 * kv_hash - compute hash for key
 * @key: key string
 *
 * Return: hash value
 */
static uint32_t kv_store_hash(const char *key)
{
  uint32_t hash = 5381;
  int c;

  while ((c = *key++))
    hash = ((hash << 5) + hash) + c;

  return hash & (KV_HASH_SIZE - 1);
}

/**
 * kv_node_free - free node and its resources
 * @node: node to free
 */
void kv_node_free(struct kv_node *node)
{
  if (!node)
    return;

  free(node->key);
  memset(node, 0, sizeof(*node));
  free(node);
}

/**
 * kv_store_lookup - find node by key
 * @store: store instance
 * @key: key to search for
 *
 * Return: node if found, NULL otherwise
 */
struct kv_node *kv_store_lookup(const struct kv_store *store, const char *key)
{
  uint32_t hash;
  struct kv_node *node;

  if (!store || !key)
    return NULL;

  hash = kv_store_hash(key);

  node = store->buckets[hash];

  while (node) {
    if (strcmp(key, node->key) == 0)
      return node;

    node = node->next;
  }

  return NULL;
}

/**
 * kv_store_init - initialize store
 * @store: store to initialize
 *
 * Return: 0 on success, -EINVAL on failure
 */
int kv_store_init(struct kv_store *store)
{
  if (!store)
    return -EINVAL;

  memset(store, 0, sizeof(*store));

  return 0;
}

/**
 * kv_store_destroy - destroy store and free all entries
 * @store: store to destroy
 */
void kv_store_destroy(struct kv_store *store)
{
  int i;
  struct kv_node *node, *tmp;

  if (!store)
    return;

  for (i = 0; i < store->count; ++i) {
    node = store->buckets[i];
    while (node) {
      tmp = node->next;
      kv_node_free(node);
      node = tmp;
    }
  }

  memset(store, 0, sizeof(*store));
}

/**
 * kv_store_put - insert or update key-value pair
 * @store: store instance
 * @key: key string (sensor_id)
 * @data: sensor data payload
 *
 * Return: 0 on success, negative error code on failure
 */
int kv_store_put(struct kv_store *store, const char *key, const SensorData *data)
{
  struct kv_node *node, *new_node;
  int hash;

  if (!store || !key || !data)
    return -EINVAL;

  pthread_mutex_lock(&kv_mutex);

  node = kv_store_lookup(store, key);
  if (node) {
    node->data = *data;
    pthread_mutex_unlock(&kv_mutex);
    return 0;
  }

  new_node = malloc(sizeof(*new_node));
  if (!new_node) {
    pthread_mutex_unlock(&kv_mutex);
    goto err_alloc_node;
  }

  new_node->key = strdup(key);
  if (!new_node->key) {
    free(new_node);
    pthread_mutex_unlock(&kv_mutex);
    goto err_alloc_key;
  }

  new_node->data = *data;

  hash = kv_store_hash(key);
  new_node->next = store->buckets[hash];
  store->buckets[hash] = new_node;
  store->count += 1;

  pthread_mutex_unlock(&kv_mutex);
  return 0;

 err_alloc_key:
  perror("err_alloc_key");
  return -ENOMEM;
 err_alloc_node:
  perror("err_alloc_node");
  return -ENOMEM;
}

/**
 * kv_store_get - retrieve sensor data by key
 * @store: store instance
 * @key: key to lookup (sensor_id)
 * @value: output SensorData pointer
 *
 * Return: 0 on success, -ENOENT if not found, -EINVAL on invalid args
 */
int kv_store_get(struct kv_store *store, const char *key, SensorData *value)
{
  struct kv_node *node;

  if (!store || !key || !value)
    return -EINVAL;

  pthread_mutex_lock(&kv_mutex);

  node = kv_store_lookup(store, key);
  if (!node) {
    pthread_mutex_unlock(&kv_mutex);
    return -ENOENT;
  }

  *value = node->data;
  pthread_mutex_unlock(&kv_mutex);
  return 0;
}

/**
 * kv_store_delete - remove entry by key
 * @store: store instance
 * @key: key to delete
 *
 * Return: 0 on success, -ENOENT if not found
 */
int kv_store_delete(struct kv_store *store, const char *key)
{
  uint32_t hash;
  struct kv_node *node, *prev;

  if (!store || !key)
    return -EINVAL;

  pthread_mutex_lock(&kv_mutex);

  hash = kv_store_hash(key);

  node = store->buckets[hash];
  prev = NULL;

  while (node) {
    if (strcmp(key, node->key) == 0) {
      if (prev)
        prev->next = node->next;
      else
        store->buckets[hash] = node->next;

      kv_node_free(node);

      store->count -= 1;
      pthread_mutex_unlock(&kv_mutex);
      return 0;
    }

    prev = node;
    node = node->next;
  }

  pthread_mutex_unlock(&kv_mutex);
  return -ENOENT;
}

/**
 * kv_store_exists - check if key exists
 * @store: store instance
 * @key: key to check
 *
 * Return: true if exists, false otherwise
 */
bool kv_store_exists(struct kv_store *store, const char *key)
{
  if (!store || !key)
    return false;

  return kv_store_lookup(store, key) != NULL;
}

/**
 * kv_store_dump - print all entries (debugging)
 * @store: store instance
 */
void kv_store_dump(struct kv_store *store)
{
  int i;
  struct kv_node *node;

  if (!store)
    return;

  pthread_mutex_lock(&kv_mutex);

  printf("\n=== Device Store Dump (%d devices) ===\n", store->count);
  
  for (i = 0; i < KV_HASH_SIZE; i++) {
    node = store->buckets[i];
    while (node) {
      printf("ID=%s Temp=%.2f°C Hum=%.2f%% AvgT=%.2f°C AvgH=%.2f%% Time=%s\n",
             node->data.sensor_id,
             node->data.temperature,
             node->data.humidity,
             node->data.avgTemperature,
             node->data.avgHumidity,
             node->data.timestamp);
      node = node->next;
    }
  }
  
  printf("=== End Store Dump ===\n\n");
  pthread_mutex_unlock(&kv_mutex);
}
