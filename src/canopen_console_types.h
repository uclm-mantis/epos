#ifndef CT
#define CT(name, t, parserfmt, printfmt)
#endif
#ifndef CTA
#define CTA(name, t, parserfmt, printfmt) CT(name, t, parserfmt, printfmt)
#endif
#ifndef CTS
#define CTS(name, t)
#endif

CT (b,   bool,     ul,  d)
CT (i8,  int8_t,   l,   d)
CT (u8,  uint8_t,  ul,  u)
CTA(x8,  uint8_t,  ul,  02x)
CT (i16, int16_t,  l,   d)
CT (u16, uint16_t, ul,  u)
CTA(x16, uint16_t, ul,  04x)
CT (i32, int32_t,  l,   d)
CT (u32, uint32_t, ul,  u)
CTA(x32, uint32_t, ul,  08x)
CT (i64, int64_t,  ll,  lld)
CT (u64, uint64_t, ull, llu)
CTA(x64, uint64_t, ull, 016x)
CTS(vs,  string8_t)

#undef CT
#undef CTA
#undef CTS
