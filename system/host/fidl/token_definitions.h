// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// No header inclusion guards _sic_ as this may be re-included with
// different X macro arguments.

#if !defined(TOKEN)
#define TOKEN(Name)
#endif

#if !defined(KEYWORD)
#define KEYWORD(Name, Spelling) TOKEN(Name)
#endif

// Control and whitespace
TOKEN(NotAToken)
TOKEN(EndOfFile)
TOKEN(Comment)

// Identifiers and constants
TOKEN(Identifier)
TOKEN(NumericLiteral)
TOKEN(StringLiteral)

// Punctuation.
TOKEN(LeftParen)
TOKEN(RightParen)
TOKEN(LeftSquare)
TOKEN(RightSquare)
TOKEN(LeftCurly)
TOKEN(RightCurly)
TOKEN(LeftAngle)
TOKEN(RightAngle)

TOKEN(Dot)
TOKEN(Comma)
TOKEN(Semicolon)
TOKEN(Colon)
TOKEN(Question)
TOKEN(Equal)
TOKEN(Ampersand)

TOKEN(Arrow)

// Keywords
KEYWORD(As, "as")
KEYWORD(Module, "module")
KEYWORD(Using, "using")

KEYWORD(Handle, "handle")
KEYWORD(Request, "request")

KEYWORD(Const, "const")
KEYWORD(Enum, "enum")
KEYWORD(Interface, "interface")
KEYWORD(Struct, "struct")
KEYWORD(Union, "union")

KEYWORD(String, "string")
KEYWORD(Bool, "bool")
KEYWORD(Int8, "int8")
KEYWORD(Int16, "int16")
KEYWORD(Int32, "int32")
KEYWORD(Int64, "int64")
KEYWORD(Uint8, "uint8")
KEYWORD(Uint16, "uint16")
KEYWORD(Uint32, "uint32")
KEYWORD(Uint64, "uint64")
KEYWORD(Float32, "float32")
KEYWORD(Float64, "float64")

KEYWORD(True, "true")
KEYWORD(False, "false")
KEYWORD(Default, "default")

#undef KEYWORD
#undef TOKEN
