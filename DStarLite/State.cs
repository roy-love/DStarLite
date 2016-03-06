using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DStarLite
{
    class State
    {
        public int x;
        public int y;
        
        /// <summary>
        /// The state is a combination of this class and StateInfo.
        /// </summary>
        /// <param name="x">The x coordinate.</param>
        /// <param name="y">The y coordinate.</param>
        public State(int x, int y)
        {
            this.x = x;
            this.y = y;
        }

        public override bool Equals(object obj)
        {
            if (this == obj)
            {
                return true;
            }
            var item = obj as State;
            if (item == null)
            {
                return false;
            }
            return (x == item.x && y == item.y);
        }

        public override int GetHashCode()
        {
            int hash = 3;
            hash = 79 * hash + this.x;
            hash = 79 * hash + this.y;
            return hash;
        }
    }
}