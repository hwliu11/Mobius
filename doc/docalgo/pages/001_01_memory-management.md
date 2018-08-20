Memory management {#memmgt}
===========================

Fundamental algorithms in _mobiusBSpl_ and _mobiusGeom_ libraries often work with a dynamically allocated memory (heap).
Intensive computations tend to have hotspots in memory allocation/deallocation instructions which slow down the entire performance
of the algorithms. To fix that generic problem, software libraries often implement custom memory management patterns.
Following the idea to minimize heap allocations and reuse the already allocated memory blocks as much as possible, we have
developed a couple of well-working yet straightforward tools.

In Mobius, the memory-critical routines, such as B-spline or geometry evaluators provide an interface to pass externally-defined memory allocators.
The allocators are implemented in classes mobius::core_HeapAlloc and mobius::core_HeapAlloc2D. The instances of both classes allocate
a heap memory and release it _automatically_ in their destructors thus ensuring no memory leaks. The allocators themselves can be
created either in a stack memory or a heap. In the latter case, the right idea is to use a smart pointer to manage their
lifetime. The following code snippet illustrates a practical scenario of using a two-dimensional heap allocator in mobius::geom_FairBCurve.

    // Prepare reusable memory blocks for running sub-routines efficiently.
    ptr<alloc2d> sharedAlloc = new alloc2d;
    sharedAlloc->Allocate(3,     p + 1, true); // memBlock_EffectiveNDersResult
    sharedAlloc->Allocate(2,     3,     true); // memBlock_EffectiveNDersInternal
    sharedAlloc->Allocate(p + 1, p + 1, true); // memBlock_BSplineCurveEvalDk

The type `alloc2d` is a convenience alias for mobius::core_HeapAlloc2D. This code creates a shared allocator of two-dimensional memory blocks.
Then three blocks of different (dynamic) dimensions are allocated. An optional `true` flag forces the allocator to nullify
all elements (such initialization has a certain performance overhead). Then, in other sub-routines which accept the same allocator,
the once prepared memory blocks can be reused. The following code excerpt shows how the allocator argument is treated within
mobius::geom_BSplineCurve.

    void mobius::geom_BSplineCurve::Eval_Dk(const double u,
                                            const int    k,
                                            xyz&         dkC,
                                            ptr<alloc2d> alloc,
                                            const int    memBlockResult,
                                            const int    memBlockInternal) const
    {
      ptr<alloc2d> localAlloc;

      double** dN;
      if ( alloc.IsNull() )
      {
        localAlloc = new alloc2d;
        dN = localAlloc->Allocate(m_iDeg + 1, m_iDeg + 1, true);
      }
      else
        dN = alloc->Access(memBlockResult).Ptr;

        ...
    }

In case if an allocator instance is passed, the existing memory block will be taken by a specific index.
If no allocator is passed, the curve object creates all necessary resources locally.
