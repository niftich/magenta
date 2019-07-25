#ifndef JEMALLOC_INTERNAL_WITNESS_EXTERNS_H
#define JEMALLOC_INTERNAL_WITNESS_EXTERNS_H

#pragma GCC visibility push(hidden)

void	witness_init(witness_t *witness, const char *name, witness_rank_t rank,
    witness_comp_t *comp, void *opaque);
#ifdef JEMALLOC_JET
typedef void (witness_lock_error_t)(const witness_list_t *, const witness_t *);
extern witness_lock_error_t *witness_lock_error;
#else
void	witness_lock_error(const witness_list_t *witnesses,
    const witness_t *witness);
#endif
#ifdef JEMALLOC_JET
typedef void (witness_owner_error_t)(const witness_t *);
extern witness_owner_error_t *witness_owner_error;
#else
void	witness_owner_error(const witness_t *witness);
#endif
#ifdef JEMALLOC_JET
typedef void (witness_not_owner_error_t)(const witness_t *);
extern witness_not_owner_error_t *witness_not_owner_error;
#else
void	witness_not_owner_error(const witness_t *witness);
#endif
#ifdef JEMALLOC_JET
typedef void (witness_lockless_error_t)(const witness_list_t *);
extern witness_lockless_error_t *witness_lockless_error;
#else
void	witness_lockless_error(const witness_list_t *witnesses);
#endif

void	witnesses_cleanup(tsd_t *tsd);
void	witness_prefork(tsd_t *tsd);
void	witness_postfork_parent(tsd_t *tsd);
void	witness_postfork_child(tsd_t *tsd);

#pragma GCC visibility pop

#endif /* JEMALLOC_INTERNAL_WITNESS_EXTERNS_H */
