package Debugging;

import android.util.Log;

import java.io.ObjectOutputStream;
import java.net.InetAddress;
import java.net.Socket;
import java.util.ArrayList;
import java.util.concurrent.Semaphore;

public class TcpServer implements Runnable{

    //this will kill the process if set to true
    public static boolean kill = false;


    //this is the port we are communicating on
    private int clientPort = 6666;

    //this is the address of the laptop
    private String laptopAddress = "192.168.49.194";

    //guards thread collisions
    private Semaphore sendLock = new Semaphore(1);

    //this is the list of messages to send
    private ArrayList<String> sendMe = new ArrayList<>();




    //initialize a TcpServer with the port
    public TcpServer(int clientPort) {
        this.clientPort = clientPort;//set our client port
    }

    @Override
    public void run() {
        Log.d("TCPLOG","Running tcpip server");

        try{
            //initialize the InetAddress to the laptop address
            InetAddress host = InetAddress.getByName("192.168.49.194");
            Log.d("TCPLOG","got host address");


            //this is our socket we will be sending on
            Socket socket;

            ObjectOutputStream outputStream;



            for(;;){

                sendLock.acquire();//wait to acquire the semaphore (blocking)

                Log.d("TCPLOG","acquired semaphore");

                //clip send me so that only the last update of info is sent
                clipSendMe();

                Log.d("TCPLOG","clipped sendMe");


                //go through all the messages
                for(int i = 0; i < sendMe.size(); i ++){
                    //establish socket connection to server
                    socket = new Socket(host.getHostAddress(), clientPort);
                    Log.d("TCPLOG","initialized socket");

                    //write to socket using ObjectOutputStream
                    outputStream = new ObjectOutputStream(socket.getOutputStream());

                    Log.d("TCPLOG","initialized outputStream");

                    outputStream.writeObject(sendMe.get(i));

                    outputStream.close();
                }

                sendMe.clear();
                //now release the semaphore
                sendLock.release();


//                Thread.sleep(100);

            }

        }catch(Exception e){
            e.printStackTrace();
        }

    }

    /**
     * This will clip out all the data in the buffer that is out of date
     * Keeping only the last update
     */
    private void clipSendMe() {
        if(sendMe.size() == 0){return;}//return if the size is 0

        //This buffer could have stuff we don't need to send
        //so we will first find the last occurrance of "END"
        //and flag that as the start point for sending
        int indexOfLatestMessageStart = 0;
        for(int i = sendMe.size()-1; i >= 0; i --){
            if(sendMe.get(i).equals("CLEAR")){
                indexOfLatestMessageStart = i;
                break;//we are done now, exit the loop
            }
        }



        //this will now clip the list to only be the stuff we need to send (the last update)
//        sendMe = (ArrayList<String>)
//                sendMe.subList(indexOfLatestMessageStart, sendMe.size());
        for(int i = indexOfLatestMessageStart-1; i >=0; i --){
            sendMe.remove(i);
        }
    }

    //this will hold messages if they can't be sent yet
    private ArrayList<String> tempBuffer = new ArrayList<>();


    /**
     * Adds a message to the buffer to be sent if possible.
     * @param string message here
     */
    public void addMessage(String string){
        //don't add if we are already sending
        if(!sendLock.tryAcquire()){
            tempBuffer.add(string);
            return;
        }else{

            //add all the temp buffer to the real buffer
            sendMe.addAll(tempBuffer);
            tempBuffer.clear();//clear the temp buffer
            sendMe.add(string);
            sendLock.release();
        }

    }


    /**
     * This method will mark the end point of this update and clear the stuff we don't need
     */
    public void markEndUpdate() {
        addMessage("CLEAR");
    }
}
