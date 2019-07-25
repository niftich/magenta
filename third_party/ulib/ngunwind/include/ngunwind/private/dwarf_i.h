#ifndef DWARF_I_H
#define DWARF_I_H

/* This file contains definitions that cannot be used in code outside
   of libunwind.  In particular, most inline functions are here
   because otherwise they'd generate unresolved references when the
   files are compiled with inlining disabled.  */

#include "dwarf.h"
#include "libunwind_i.h"

#define DWARF_GET_LOC(l)        ((l).val)

# define DWARF_LOC_TYPE_MEM     (0 << 0)
# define DWARF_LOC_TYPE_FP      (1 << 0)
# define DWARF_LOC_TYPE_REG     (1 << 1)
# define DWARF_LOC_TYPE_VAL     (1 << 2)

# define DWARF_IS_REG_LOC(l)    (((l).type & DWARF_LOC_TYPE_REG) != 0)
# define DWARF_IS_FP_LOC(l)     (((l).type & DWARF_LOC_TYPE_FP) != 0)
# define DWARF_IS_MEM_LOC(l)    ((l).type == DWARF_LOC_TYPE_MEM)
# define DWARF_IS_VAL_LOC(l)    (((l).type & DWARF_LOC_TYPE_VAL) != 0)

# define DWARF_LOC(r, t)        ((dwarf_loc_t) { .val = (r), .type = (t) })
# define DWARF_VAL_LOC(c,v)     DWARF_LOC ((v), DWARF_LOC_TYPE_VAL)
# define DWARF_MEM_LOC(c,m)     DWARF_LOC ((m), DWARF_LOC_TYPE_MEM)

# define DWARF_NULL_LOC         DWARF_LOC (0, 0)
# define DWARF_IS_NULL_LOC(l)                                           \
                ({ dwarf_loc_t _l = (l); _l.val == 0 && _l.type == 0; })
# define DWARF_REG_LOC(c,r)     DWARF_LOC((r), DWARF_LOC_TYPE_REG)
# define DWARF_FPREG_LOC(c,r)   DWARF_LOC((r), (DWARF_LOC_TYPE_REG      \
                                                | DWARF_LOC_TYPE_FP))

static inline int
dwarf_get (struct dwarf_cursor *c, dwarf_loc_t loc, unw_word_t *val)
{
  if (DWARF_IS_NULL_LOC (loc))
    return -UNW_EBADREG;

  /* If a code-generator were to save a value of type unw_word_t in a
     floating-point register, we would have to support this case.  I
     suppose it could happen with MMX registers, but does it really
     happen?  */
  assert (!DWARF_IS_FP_LOC (loc));

#if UNW_TARGET_ARM || UNW_TARGET_AARCH64
  assert (DWARF_IS_REG_LOC (loc) || DWARF_IS_MEM_LOC (loc));
#endif

  if (DWARF_IS_REG_LOC (loc))
    return (*c->as->acc.access_reg) (c->as, (unw_regnum_t) DWARF_GET_LOC (loc), val,
                                     0, c->as_arg);
  if (DWARF_IS_MEM_LOC (loc))
    return (*c->as->acc.access_mem) (c->as, DWARF_GET_LOC (loc), val,
                                     0, c->as_arg);

  assert(DWARF_IS_VAL_LOC (loc));
  *val = DWARF_GET_LOC (loc);
  return 0;
}

static inline int
dwarf_put (struct dwarf_cursor *c, dwarf_loc_t loc, unw_word_t val)
{
  assert(!DWARF_IS_VAL_LOC (loc));

  if (DWARF_IS_NULL_LOC (loc))
    return -UNW_EBADREG;

  /* If a code-generator were to save a value of type unw_word_t in a
     floating-point register, we would have to support this case.  I
     suppose it could happen with MMX registers, but does it really
     happen?  */
  assert (!DWARF_IS_FP_LOC (loc));

  if (DWARF_IS_REG_LOC (loc))
    return (*c->as->acc.access_reg) (c->as, (unw_regnum_t) DWARF_GET_LOC (loc), &val,
                                     1, c->as_arg);
  else
    return (*c->as->acc.access_mem) (c->as, DWARF_GET_LOC (loc), &val,
                                     1, c->as_arg);
}

/* Unless we are told otherwise, assume that a "machine address" is
   the size of an unw_word_t.  */
#ifndef dwarf_addr_size
# define dwarf_addr_size(as) (sizeof (unw_word_t))
#endif

#ifndef dwarf_to_unw_regnum
extern const uint8_t dwarf_to_unw_regnum_map[DWARF_REGNUM_MAP_LENGTH];
/* REG is evaluated multiple times; it better be side-effects free!  */
# define dwarf_to_unw_regnum(reg)                                         \
  (((reg) < DWARF_REGNUM_MAP_LENGTH) ? dwarf_to_unw_regnum_map[reg] : 0)
#endif

static inline int
dwarf_readu8 (unw_addr_space_t as, unw_accessors_t *a, unw_word_t *addr,
              uint8_t *valp, void *arg)
{
  if (a->access_raw_mem)
    {
      int ret = (*a->access_raw_mem) (as, *addr, valp, 1, 0, arg);
      *addr += 1;
      return ret;
    }
  else
    {
      unw_word_t val, aligned_addr = *addr & -sizeof (unw_word_t);
      unw_word_t off = *addr - aligned_addr;
      int ret;

      *addr += 1;
      ret = (*a->access_mem) (as, aligned_addr, &val, 0, arg);
#if __BYTE_ORDER == __LITTLE_ENDIAN
      val >>= 8*off;
#else
      val >>= 8*(sizeof (unw_word_t) - 1 - off);
#endif
      *valp = (uint8_t) val;
      return ret;
    }
}

static inline int
dwarf_readu16 (unw_addr_space_t as, unw_accessors_t *a, unw_word_t *addr,
               uint16_t *val, void *arg)
{
  if (a->access_raw_mem)
    {
      uint16_t tval;
      int ret = (*a->access_raw_mem) (as, *addr, &tval, 2, 0, arg);
      if ((__BYTE_ORDER == __BIG_ENDIAN) == tdep_big_endian (as))
        *val = tval;
      else
        *val = (tval >> 8) | ((tval & 0xff) << 8);
      *addr += 2;
      return ret;
    }
  else
    {
      uint8_t v0 = 0, v1 = 0;
      int ret;

      if ((ret = dwarf_readu8 (as, a, addr, &v0, arg)) < 0
          || (ret = dwarf_readu8 (as, a, addr, &v1, arg)) < 0)
        return ret;

      if (tdep_big_endian (as))
        *val = (uint16_t) v0 << 8 | v1;
      else
        *val = (uint16_t) v1 << 8 | v0;
      return 0;
    }
}

static inline uint32_t
byteswap32 (uint32_t val)
{
  return ((val >> 24)
          | ((val >> 16) & 0xff00)
          | ((val & 0xff00) << 8)
          | ((val & 0xff) << 24));
}

static inline int
dwarf_readu32 (unw_addr_space_t as, unw_accessors_t *a, unw_word_t *addr,
               uint32_t *val, void *arg)
{
  if (a->access_raw_mem)
    {
      uint32_t tval;
      int ret = (*a->access_raw_mem) (as, *addr, &tval, 4, 0, arg);
      if ((__BYTE_ORDER == __BIG_ENDIAN) == tdep_big_endian (as))
        *val = tval;
      else
        *val = byteswap32 (tval);
      *addr += 4;
      return ret;
    }
  else
    {
      uint16_t v0 = 0, v1 = 0;
      int ret;

      if ((ret = dwarf_readu16 (as, a, addr, &v0, arg)) < 0
          || (ret = dwarf_readu16 (as, a, addr, &v1, arg)) < 0)
        return ret;

      if (tdep_big_endian (as))
        *val = (uint32_t) v0 << 16 | v1;
      else
        *val = (uint32_t) v1 << 16 | v0;
      return 0;
    }
}

static inline int
dwarf_readu64 (unw_addr_space_t as, unw_accessors_t *a, unw_word_t *addr,
               uint64_t *val, void *arg)
{
  if (a->access_raw_mem)
    {
      uint64_t tval;
      int ret = (*a->access_raw_mem) (as, *addr, &tval, 8, 0, arg);
      if ((__BYTE_ORDER == __BIG_ENDIAN) == tdep_big_endian (as))
        *val = tval;
      else
        *val = (((uint64_t) byteswap32 (tval) << 32)
                | byteswap32 (tval >> 32));
      *addr += 8;
      return ret;
    }
  else
    {
      uint32_t v0 = 0, v1 = 0;
      int ret;

      if ((ret = dwarf_readu32 (as, a, addr, &v0, arg)) < 0
          || (ret = dwarf_readu32 (as, a, addr, &v1, arg)) < 0)
        return ret;

      if (tdep_big_endian (as))
        *val = (uint64_t) v0 << 32 | v1;
      else
        *val = (uint64_t) v1 << 32 | v0;
      return 0;
    }
}

static inline int
dwarf_reads8 (unw_addr_space_t as, unw_accessors_t *a, unw_word_t *addr,
              int8_t *val, void *arg)
{
  uint8_t uval = 0;
  int ret;

  if ((ret = dwarf_readu8 (as, a, addr, &uval, arg)) < 0)
    return ret;
  *val = (int8_t) uval;
  return 0;
}

static inline int
dwarf_reads16 (unw_addr_space_t as, unw_accessors_t *a, unw_word_t *addr,
               int16_t *val, void *arg)
{
  uint16_t uval = 0;
  int ret;

  if ((ret = dwarf_readu16 (as, a, addr, &uval, arg)) < 0)
    return ret;
  *val = (int16_t) uval;
  return 0;
}

static inline int
dwarf_reads32 (unw_addr_space_t as, unw_accessors_t *a, unw_word_t *addr,
               int32_t *val, void *arg)
{
  uint32_t uval = 0;
  int ret;

  if ((ret = dwarf_readu32 (as, a, addr, &uval, arg)) < 0)
    return ret;
  *val = (int32_t) uval;
  return 0;
}

static inline int
dwarf_reads64 (unw_addr_space_t as, unw_accessors_t *a, unw_word_t *addr,
               int64_t *val, void *arg)
{
  uint64_t uval = 0;
  int ret;

  if ((ret = dwarf_readu64 (as, a, addr, &uval, arg)) < 0)
    return ret;
  *val = (int64_t) uval;
  return 0;
}

static inline int
dwarf_readw (unw_addr_space_t as, unw_accessors_t *a, unw_word_t *addr,
             unw_word_t *val, void *arg)
{
  uint32_t u32;
  uint64_t u64;
  int ret;

  switch (dwarf_addr_size (as))
    {
    case 4:
      ret = dwarf_readu32 (as, a, addr, &u32, arg);
      if (ret < 0)
        return ret;
      *val = u32;
      return ret;

    case 8:
      ret = dwarf_readu64 (as, a, addr, &u64, arg);
      if (ret < 0)
        return ret;
      *val = u64;
      return ret;

    default:
      assert (0);
    }
}

/* Read an unsigned "little-endian base 128" value.  See Chapter 7.6
   of DWARF spec v3.  */

static inline int
dwarf_read_uleb128 (unw_addr_space_t as, unw_accessors_t *a, unw_word_t *addr,
                    unw_word_t *valp, void *arg)
{
  unw_word_t val = 0, shift = 0;
  unsigned char byte;
  int ret;

  do
    {
      if ((ret = dwarf_readu8 (as, a, addr, &byte, arg)) < 0)
        return ret;

      val |= ((unw_word_t) byte & 0x7f) << shift;
      shift += 7;
    }
  while (byte & 0x80);

  *valp = val;
  return 0;
}

/* Read a signed "little-endian base 128" value.  See Chapter 7.6 of
   DWARF spec v3.  */

static inline int
dwarf_read_sleb128 (unw_addr_space_t as, unw_accessors_t *a, unw_word_t *addr,
                    unw_word_t *valp, void *arg)
{
  unw_word_t val = 0, shift = 0;
  unsigned char byte;
  int ret;

  do
    {
      if ((ret = dwarf_readu8 (as, a, addr, &byte, arg)) < 0)
        return ret;

      val |= ((unw_word_t) byte & 0x7f) << shift;
      shift += 7;
    }
  while (byte & 0x80);

  if (shift < 8 * sizeof (unw_word_t) && (byte & 0x40) != 0)
    /* sign-extend negative value */
    val |= ((unw_word_t) -1) << shift;

  *valp = val;
  return 0;
}

static ALWAYS_INLINE int
dwarf_read_encoded_pointer_inlined (unw_addr_space_t as, unw_accessors_t *a,
                                    unw_word_t *addr, unsigned char encoding,
                                    const unw_proc_info_t *pi,
                                    unw_word_t *valp, void *arg)
{
  unw_word_t val, initial_addr = *addr;
  uint16_t uval16;
  uint32_t uval32;
  uint64_t uval64;
  int16_t sval16;
  int32_t sval32;
  int64_t sval64;
  int ret;

  /* DW_EH_PE_omit and DW_EH_PE_aligned don't follow the normal
     format/application encoding.  Handle them first.  */
  if (encoding == DW_EH_PE_omit)
    {
      *valp = 0;
      return 0;
    }
  else if (encoding == DW_EH_PE_aligned)
    {
      int size = dwarf_addr_size (as);
      *addr = (initial_addr + size - 1) & -size;
      return dwarf_readw (as, a, addr, valp, arg);
    }

  switch (encoding & DW_EH_PE_FORMAT_MASK)
    {
    case DW_EH_PE_ptr:
      if ((ret = dwarf_readw (as, a, addr, &val, arg)) < 0)
        return ret;
      break;

    case DW_EH_PE_uleb128:
      if ((ret = dwarf_read_uleb128 (as, a, addr, &val, arg)) < 0)
        return ret;
      break;

    case DW_EH_PE_udata2:
      uval16 = 0;
      if ((ret = dwarf_readu16 (as, a, addr, &uval16, arg)) < 0)
        return ret;
      val = uval16;
      break;

    case DW_EH_PE_udata4:
      uval32 = 0;
      if ((ret = dwarf_readu32 (as, a, addr, &uval32, arg)) < 0)
        return ret;
      val = uval32;
      break;

    case DW_EH_PE_udata8:
      uval64 = 0;
      if ((ret = dwarf_readu64 (as, a, addr, &uval64, arg)) < 0)
        return ret;
      val = uval64;
      break;

    case DW_EH_PE_sleb128:
      if ((ret = dwarf_read_uleb128 (as, a, addr, &val, arg)) < 0)
        return ret;
      break;

    case DW_EH_PE_sdata2:
      sval16 = 0;
      if ((ret = dwarf_reads16 (as, a, addr, &sval16, arg)) < 0)
        return ret;
      val = sval16;
      break;

    case DW_EH_PE_sdata4:
      sval32 = 0;
      if ((ret = dwarf_reads32 (as, a, addr, &sval32, arg)) < 0)
        return ret;
      val = sval32;
      break;

    case DW_EH_PE_sdata8:
      sval64 = 0;
      if ((ret = dwarf_reads64 (as, a, addr, &sval64, arg)) < 0)
        return ret;
      val = sval64;
      break;

    default:
      Debug (1, "unexpected encoding format 0x%x\n",
             encoding & DW_EH_PE_FORMAT_MASK);
      return -UNW_EINVAL;
    }

  if (val == 0)
    {
      /* 0 is a special value and always absolute.  */
      *valp = 0;
      return 0;
    }

  switch (encoding & DW_EH_PE_APPL_MASK)
    {
    case DW_EH_PE_absptr:
      break;

    case DW_EH_PE_pcrel:
      val += initial_addr;
      break;

    case DW_EH_PE_datarel:
      /* XXX For now, assume that data-relative addresses are relative
         to the global pointer.  */
      val += pi->gp;
      break;

    case DW_EH_PE_funcrel:
      val += pi->start_ip;
      break;

    case DW_EH_PE_textrel:
      /* XXX For now we don't support text-rel values.  If there is a
         platform which needs this, we probably would have to add a
         "segbase" member to unw_proc_info_t.  */
    default:
      Debug (1, "unexpected application type 0x%x\n",
             encoding & DW_EH_PE_APPL_MASK);
      return -UNW_EINVAL;
    }

  /* Trim off any extra bits.  Assume that sign extension isn't
     required; the only place it is needed is MIPS kernel space
     addresses.  */
  if (sizeof (val) > dwarf_addr_size (as))
    {
      assert (dwarf_addr_size (as) == 4);
      val = (uint32_t) val;
    }

  if (encoding & DW_EH_PE_indirect)
    {
      unw_word_t indirect_addr = val;

      if ((ret = dwarf_readw (as, a, &indirect_addr, &val, arg)) < 0)
        return ret;
    }

  *valp = val;
  return 0;
}

#endif /* DWARF_I_H */
