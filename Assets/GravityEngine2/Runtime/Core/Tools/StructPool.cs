using Unity.Collections;
using System.Collections.Generic;

namespace GravityEngine2 {
    /// <summary>
    /// Utility to manage a pool of structs in GECore
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public class StructPool<T> where T : struct {
        private Queue<int> freeList;
        private int growBy;

        /// <summary>
        /// Default constructor
        /// </summary>
        public StructPool()
        {
            freeList = new Queue<int>();
            growBy = 0;
        }

        /// <summary>
        /// Copy constructor - creates a deep copy of the source pool
        /// </summary>
        /// <param name="source">The source pool to copy from</param>
        public StructPool(StructPool<T> source)
        {
            if (source == null) {
                freeList = new Queue<int>();
                growBy = 0;
            } else {
                growBy = source.growBy;
                freeList = new Queue<int>(source.freeList);
            }
        }

        public void Init(NativeArray<T> a, int growBy)
        {
            int size = a.Length;
            this.growBy = growBy;
            freeList = new Queue<int>();
            for (int i = 0; i < size; i++)
                freeList.Enqueue(i);
        }

        public int Alloc(ref NativeArray<T> a)
        {
            if (freeList.Count == 0) {
                // need to grow the array
                NativeArray<T> a_old = a;
                a = new NativeArray<T>(a_old.Length + growBy, Allocator.Persistent);
                // cannot use copy from, requires exact length match
                for (int i = 0; i < a_old.Length; i++)
                    a[i] = a_old[i];
                for (int i = a_old.Length; i < a.Length; i++)
                    freeList.Enqueue(i);
            }
            return freeList.Dequeue();
        }

        public void Free(int index)
        {
            freeList.Enqueue(index);
        }

    }
}
