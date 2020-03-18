var config = {
    type: Phaser.WEBGL,
    width: 800,
    height: 600,
    backgroundColor: '#2d2d2d',
    pixelArt: true,
    parent: 'phaser-example',
    scene: {
        preload: preload,
        create: create,
        update:update
    }
};



var game = new Phaser.Game(config);
var text;
var tween;
var tween3;
var tweens_array=[];
function preload ()
{
    this.load.image('cokecan', 'assets/sprites/cokecan.png');
}

function create ()
{
    var marker = this.add.image(100, 100, 'cokecan').setAlpha(0.3);
    var image = this.add.image(100, 100, 'cokecan');
var image2 = this.add.image(200, 100, 'cokecan');
var image3 = this.add.image(300, 500, 'cokecan');
    //  flipY will call toggleFlipY on the image whenever it yoyos or repeats

    // tween = this.tweens.add({
    //     targets: [image,image2,image3],
    //       y:500,
       
    // //    // ease: 'Sine.easeInOut',
    // //     yoyo: true,
    // //      duration: 1500,
    // //     repeat: -1,
    // //     delay: this.tweens.stagger(50)

    //     yoyo: true,
    //     duration: 2000,
    //     ease: 'Sine.easeInOut',
    //     repeat: -1,
    //   delay: 100
    // });
         this.tweens.add({
        targets: image,
               y:500,
        duration: 1500,
        yoyo: true,
        repeat: -1,
        delay:100
    });

   tween=this.tweens.add({
        targets: image2,
        y:500,
        duration: 1500,
        yoyo: true,
        repeat: -1,
         delay:500
    });

      tween3 =  this.tweens.add({
        targets: image3,
               y:100,
        duration: 1500,
        yoyo: true,
        repeat: -1
    });
    text = this.add.text(180, 0, 'Global timeScale: 1').setFont('32px Arial Black').setFill('#ffffff').setShadow(2, 2, "#333333", 2);
tweens_array.push(tween);
tweens_array.push(tween3);
     // tweens_array= this.tweens.getAll;
  }

function update ()
{
    debugTweenData(text, tweens_array);
}

function debugTweenData (text, tweens_array)
{
    var output = [];

    //output.push('Current: '+tween.data[0].current);
     output.push('Current: '+tweens_array[1].data[0].current);
    text.setText(output);
}
