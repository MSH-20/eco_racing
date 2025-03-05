package Final;

public class lazyEval {

	final int x=32;
	/*static {
		System.out.print(x);
	}*/
	static int y=33;
	static final int z=4;
	{
		System.out.println("instance block");
	}
	public lazyEval() {
		System.out.println("Constructor");
	}
	static double[] xArr= {2, .5};
	static final String str="static";
	static {
		System.out.println("static block");
	}
	final int yVar=3;
	public static void main(String[] arr) {
		int x = 3; 
		System.out.println((++x) * (x++));   
		System.out.println(x); 
		
		System.out.print("Hello");
	}
}

