//
// Created by dewe on 4/28/21.
//

#ifndef FASTDRL_EMPTY_ARENA_H
#define FASTDRL_EMPTY_ARENA_H


#include "arena.h"

class EmptyArena : public Arena {


public:
    explicit EmptyArena() : Arena(mjcfUtils::xmlPathCompletion("arenas/empty_arena.xml")) {}

}

#endif //FASTDRL_EMPTY_ARENA_H
