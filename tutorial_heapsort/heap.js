/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 


// define insert function for min binary heap
function minheap_insert(heap, new_element) 
{
 
  var adj,temp;
  heap.push(new_element);
  for (adj=heap.length-1;adj>0;adj=Math.floor((adj-1)/2))
  {
      if (new_element < heap[Math.floor((adj-1)/2)]) // use to swap those two numbers
        {
            temp = new_element;
            heap[adj] = heap[Math.floor((adj-1)/2)];
            heap[Math.floor((adj-1)/2)] = temp;      
        }
  }
  return;
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/
// define extract function for min binary heap
function minheap_extract(heap) 
{
  var PrintE;
  PrintE = heap[0];
  heap[0] = heap[heap.length - 1];

  var a,large;
  var temp = heap.pop();
  for (a=0;a<heap.length;a=large)
  {
      if (2*a+2>heap.length)
        {break;}
      if (heap[2*a+1] > heap[2*a+2]) // use to swap those two numbers
        {
            if (heap[a]>heap[2*a+2])
            {
              temp = heap [a];
              heap[a] = heap[2*a+2];
              heap[2*a+2] = temp; 
              large = 2*a+2;             
            }
            else
              {break;}
        }
      else
      {
         if (heap[a]>heap[2*a+1])
            {
              temp = heap [2*a+1];
              heap[2*a+1] = heap[a];
              heap[a] = temp; 
              large = 2*a+1;
            }
          else
            {break;}
      }
  }
  return PrintE;
}
// assign extract function within minheaper object
minheaper.extract = minheap_extract;
    // STENCIL: ensure extract method is within minheaper object






