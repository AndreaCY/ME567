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
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
	var p,q
	
    heap.push(new_element)
		
	for(p=heap.length-1;p>0;p=Math.floor((p-1)/2)){
		if (new_element < heap[Math.floor((p-1)/2)]){
			
			q = new_element
			heap[p] = heap[Math.floor((p-1)/2)]
			heap[Math.floor((p-1)/2)] = q                //Swap two numbers		
		}
	}
	
	return
	
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation
	
	var p,q,r
	
	Num_Extract = heap[0];
	
	heap[0] = heap[heap.length - 1];
	var temp = heap.pop();
	
	for(p=0;p<heap.length;p=r){
		
		//console.log('nimei');
		if( (2*p+1)>=heap.length){break;}
			if(heap[2*p+1]>heap[2*p+2]){
				
				if(heap[p]>heap[2*p+2]){
				
				q=heap[p]
				heap[p]=heap[2*p+2]
				heap[2*p+2]=q
					
					r=2*p+2
				}
				else{break;}
			}
			else{
				
				if(heap[p]>heap[2*p+1]){
				
				q=heap[p]
				heap[p]=heap[2*p+1]
				heap[2*p+1]=q
					
					r=2*p+1
				}
				else{break;}
			}
		
	}
	
	
	
	return Num_Extract; 
}

// assign extract function within minheaper object
minheaper.extract = minheap_extract;
    // STENCIL: ensure extract method is within minheaper object






