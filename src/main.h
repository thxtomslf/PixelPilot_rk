#pragma once

#ifndef APP_VERSION_MAJOR
#define APP_VERSION_MAJOR 1
#endif

#ifndef APP_VERSION_MINOR
#define APP_VERSION_MINOR 2
#endif

/* --- Console arguments parser --- */
#define __BeginParseConsoleArguments__(printHelpFunction) \
  if (argc < 1 || (argc == 2 && (!strcmp(argv[1], "--help") || !strcmp(argv[1], "/?") \
  || !strcmp(argv[1], "/h")))) { printHelpFunction(); return 1; } \
  for (int ArgID = 1; ArgID < argc; ArgID++) { const char* Arg = argv[ArgID];

#define __OnArgument(Name) if (!strcmp(Arg, Name))
#define __ArgValue (argc > ArgID + 1 ? argv[++ArgID] : "")
#define __EndParseConsoleArguments__ else { printf("ERROR: Unknown argument\n"); return 1; } }

// Forward declaration of external DVR thread
void *__EXTERNAL_DVR_THREAD__(void *param);