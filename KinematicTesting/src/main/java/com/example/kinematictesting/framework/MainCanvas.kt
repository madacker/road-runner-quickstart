package com.example.kinematictesting.framework

import java.awt.*
import java.awt.image.BufferStrategy
import javax.imageio.ImageIO

/**
 * Wrapper for the drawing canvas.
 */
class MainCanvas(private var internalWidth: Int, private var internalHeight: Int): java.awt.Canvas() {
    lateinit var bufferStrat: BufferStrategy

    init {
        setBounds(0, 0, internalWidth, internalHeight)
        preferredSize = Dimension(internalWidth, internalHeight)
        ignoreRepaint = true
    }

    fun start() {
        createBufferStrategy(2)
        bufferStrat = bufferStrategy

        requestFocus()
    }

    override fun getPreferredSize(): Dimension {
        return Dimension(internalWidth, internalHeight)
    }

//////////////////////////////////////////////////////////////////////////////////////

    fun getBackground(resource: String): Image {
        val classLoader = Thread.currentThread().contextClassLoader

        return ImageIO.read(classLoader.getResourceAsStream(resource))
            .getScaledInstance(internalWidth, internalHeight, Image.SCALE_SMOOTH)
    }
}
