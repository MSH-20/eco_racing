package Final;

import java.util.ArrayList;


class Piece{	
	//Elements in ArrayList are Objects
	public static ArrayList<Integer[]> PiecesLoc = new ArrayList<Integer[]>();
	
	int locX;
	int locY;
	
	public Piece(int locX, int locY)  {
		if(checkLoc(locX, locY)) {
			PiecesLoc.add(new Integer[] {locX, locY});
			
			this.locX=locX;
			this.locY=locY;
		}
		else {
			System.out.println("Place is out of boundaries or already occupied");
		}

	}
	public int getLocX() {
		return locX;
	}
	public int getLocY() {
		return locY;
	}
	
	public final boolean checkLoc(int locX, int locY) {
		boolean approved=true;
		if (locX>=0 && locX<=7 && locY>=0 && locY<=7) {
			for(Integer[] scanRow: PiecesLoc){
				if(scanRow[0]==locX && scanRow[1]==locY) {
					approved=false;
					break;
				}
			}
		}
		else {
			approved=false;
		}
		return approved;
	}	
	@Override
	public String toString() {

		return("A piece of " +this.getClass()+ " was created at: ("+
				locX+ ","+locY+")");
	}
	public static boolean removeArrayList2D(Piece pc) {
		boolean found=false;
		for(Integer[] scanRow: PiecesLoc){	        
			if(scanRow[0]==pc.getLocX() && scanRow[1]==pc.getLocY()) {
				found = true;
				PiecesLoc.remove(scanRow);
				break;
			}			
		}
		return found;
	}



public static Piece mergeCells(Piece pc1, Piece pc2) {
	
	Piece br = null;
	if(pc1.getClass().equals(pc2.getClass())
			&&(pc1.getClass().equals(Stone.class))){
	
		if (pc1.getLocX()==pc2.getLocX()
				&& (Math.abs(pc1.getLocY()-pc2.getLocY())==1))
		{
			
			if (pc1.getLocY()<pc2.getLocY())
			{										
				removeArrayList2D(pc1);
				removeArrayList2D(pc2);
				br=new Brick(pc1.getLocX(),pc1.getLocY());
				System.out.println(br.toString()+ " due to Merging");
				pc1=null;
				pc2=null;
			}
			else {
				removeArrayList2D(pc1);
				removeArrayList2D(pc2);
				br=new Brick(pc2.getLocX(),pc2.getLocY());
				System.out.println(br.toString()+ " due to Merging");
				pc1=null;
				pc2=null;
			}
		}

		else if (pc1.getLocY()==pc2.getLocY()
				&& (Math.abs(pc1.getLocX()-pc2.getLocX())==1))
		{
			
			if (pc1.getLocX()<pc2.getLocX())
			{										
				removeArrayList2D(pc1);
				removeArrayList2D(pc2);				
				br=new Brick(pc1.getLocX(),pc1.getLocY());
				System.out.println(br.toString()+ " due to Merging");
				pc1=null;
				pc2=null;
			}
			else {
				removeArrayList2D(pc1);
				removeArrayList2D(pc2);
				br=new Brick(pc2.getLocX(),pc2.getLocY());
				System.out.println(br.toString()+ " due to Merging");
				pc1=null;
				pc2=null;
			}
		}
		else
			System.out.println("Pieces are Not adjacent");
	}
	else
		System.out.println("Pieces are not similar or either/both of them is/are not Stone");

	return br;
}



}

class Stone extends Piece{

	public Stone(int locX, int locY) {
		super(locX, locY);
		// TODO Auto-generated constructor stub
	}	
}

class Brick extends Piece{

	public Brick(int locX, int locY) {
		super(locX, locY);	
		// TODO Auto-generated constructor stub
	}
}

public class FinalFall24 {

	//Merging game
	//2 stones-->brick
	//2 brick-->wall
	//Checkerboard, 8*8, indexing starts at Zero

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		Stone pc1=new Stone(2,3);
		//System.out.println(Arrays.deepToString(Piece.PiecesLoc.toArray()));
		System.out.println(pc1.toString());
		Stone pc2=new Stone(2,4);
		System.out.println(pc2.toString());
		Stone pc3=new Stone(2,7);
		System.out.println(pc3.toString());
		Brick pc4=new Brick(2,5);
		System.out.println(pc4.toString());
		if (Piece.mergeCells(pc1, pc3)!=null)
			System.out.print("Merging successful");
		if (Piece.mergeCells(pc1, pc2)!=null)
			System.out.print("Merging successful");
	
		System.gc();
	}

}
